#include <Arduino.h>

/*
 * PROGRAM KONTROL MOTOR DC: SEQUENTIAL MOTION PROFILE
 * Fitur: Soft Start, Soft Stop, Timed Run, Emergency Deceleration
 */

// --- DEKLARASI PIN ---
const int RPWM_PIN = 9;   // Kanan PWM
const int LPWM_PIN = 10;  // Kiri PWM
const int R_EN_PIN = 8;   // Enable Kanan
const int L_EN_PIN = 8;   // Enable Kiri

const int RF_A_PIN = 4; // Sequence Kiri (CCW)
const int RF_B_PIN = 5; // Sequence Kanan (CW)
const int RF_C_PIN = 6; // Continuous Run
const int RF_D_PIN = 7; // Soft STOP (Priority)

// --- KONSTANTA WAKTU (Milidetik) ---
const int RAMP_TIME = 2500; // Waktu akselerasi/decel (2.5 detik)
const int HOLD_TIME = 10000; // Waktu tahan full speed (10 detik) 2.5x2 + 10 = 15
const int MAX_PWM   = 255;  // Kecepatan Penuh

// --- VARIABEL GLOBAL ---
int currentSpeed = 0;       // Melacak kecepatan saat ini (0-255)
int currentDir = 0;         // 0: Stop, 1: Kanan (Maju), -1: Kiri (Mundur)
bool isRunningSequence = false; // Status apakah sedang menjalankan urutan otomatis

// --- PROTOTYPE FUNGSI ---
void setMotor(int speed, int direction); // Fungsi dasar penggerak
bool rampSpeed(int targetSpeed, int duration, int direction); // Akselerasi pintar
bool holdSpeed(int duration); // Menunggu pintar
void performSoftStop(); // Fungsi stop bertahap dari tombol D
void hardStop(); // Stop dadakan (safety)
void runSequence(int direction, bool autoStop); // Logika utama urutan

void setup() {
  Serial.begin(9600);
  
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  pinMode(RF_A_PIN, INPUT);
  pinMode(RF_B_PIN, INPUT);
  pinMode(RF_C_PIN, INPUT);
  pinMode(RF_D_PIN, INPUT);

  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);

  hardStop(); // Pastikan mati saat nyala
  Serial.println(F("=== SYSTEM READY: MOTION PROFILE MODE ==="));
  Serial.println(F("A: Kiri (2.5s Up -> 5s Run -> 2.5s Down)"));
  Serial.println(F("B: Kanan (2.5s Up -> 5s Run -> 2.5s Down)"));
  Serial.println(F("C: Jalan Terus (2.5s Up -> Forever)"));
  Serial.println(F("D: Soft Stop (2.5s Down dari speed sekarang)"));
}

void loop() {
  // Jika sedang tidak menjalankan sequence, baca tombol A, B, C
  if (!isRunningSequence) {
    if (digitalRead(RF_A_PIN) == HIGH) {
      Serial.println(F(">> SEQUENCE A: KIRI BERTAHAP"));
      isRunningSequence = true;
      runSequence( -1, true ); // -1 = Kiri, true = Pakai Timer Stop
    }
    else if (digitalRead(RF_B_PIN) == HIGH) {
      Serial.println(F(">> SEQUENCE B: KANAN BERTAHAP"));
      isRunningSequence = true;
      runSequence( 1, true ); // 1 = Kanan, true = Pakai Timer Stop
    }
    else if (digitalRead(RF_C_PIN) == HIGH) {
      Serial.println(F(">> SEQUENCE C: JALAN TERUS"));
      isRunningSequence = true;
      runSequence( 1, false ); // 1 = Kanan (Default), false = Jalan terus
    }
  }

  // Tombol D dicek secara khusus di luar (untuk safety tambahan jika loop utama aktif)
  // Namun, sebagian besar waktu program akan berada di dalam fungsi runSequence
  if (digitalRead(RF_D_PIN) == HIGH && currentSpeed > 0) {
    Serial.println(F(">> TOMBOL D DITEKAN: FORCED SOFT STOP!"));
    performSoftStop();
  }
}

// --- LOGIKA UTAMA SEQUENCE ---
// direction: 1 (Kanan), -1 (Kiri)
// autoStop: true (Berhenti setelah 5s), false (Jalan terus)
void runSequence(int direction, bool autoStop) {
  
  // 1. RAMP UP (0 ke 255 dalam 2.5 detik)
  Serial.println(F("Status: Ramping UP..."));
  // Jika return false, berarti ditekan D di tengah jalan -> keluar fungsi
  if (!rampSpeed(MAX_PWM, RAMP_TIME, direction)) return; 

  // 2. LOGIKA SETELAH FULL SPEED
  if (autoStop) {
    // KASUS TOMBOL A & B
    Serial.println(F("Status: Holding (5s)..."));
    if (!holdSpeed(HOLD_TIME)) return; // Cek tombol D selama nunggu
    
    // 3. RAMP DOWN (255 ke 0 dalam 2.5 detik)
    Serial.println(F("Status: Ramping DOWN..."));
    if (!rampSpeed(0, RAMP_TIME, direction)) return;

    // Selesai normal
    isRunningSequence = false; 
    hardStop();
    Serial.println(F(">> Sequence Selesai."));

  } else {
    // KASUS TOMBOL C (Jalan Terus)
    Serial.println(F("Status: Continuous Run (Tekan D untuk stop)"));
    
    // Loop idle menunggu tombol D ditekan
    // Kita menahan program di sini sampai user menekan D
    while(digitalRead(RF_D_PIN) == LOW) {
      delay(10); // Cek setiap 10ms
    }
    
    // Jika loop break (D ditekan), lakukan stop
    Serial.println(F(">> Stop Continuous Mode"));
    performSoftStop();
  }
}

// --- FUNGSI PINTAR (SMART FUNCTIONS) ---

// Fungsi Ramp yang bisa di-interrupt tombol D
// Return: true jika sukses sampai target, false jika di-interrupt tombol D
bool rampSpeed(int targetSpeed, int duration, int direction) {
  int startSpeed = currentSpeed;
  int diff = targetSpeed - startSpeed;
  
  if (diff == 0) return true;

  // Hitung delay per langkah PWM. 
  // Jika selisih 255 dan waktu 2500ms, maka delay per step ~9.8ms
  int stepDelay = duration / abs(diff); 
  if (stepDelay < 1) stepDelay = 1; 

  if (startSpeed < targetSpeed) {
    // Naik (Accelerate)
    for (int i = startSpeed; i <= targetSpeed; i++) {
      if (digitalRead(RF_D_PIN) == HIGH) { performSoftStop(); return false; } // Cek Interrupt D
      setMotor(i, direction);
      delay(stepDelay);
    }
  } else {
    // Turun (Decelerate)
    for (int i = startSpeed; i >= targetSpeed; i--) {
      if (digitalRead(RF_D_PIN) == HIGH) { performSoftStop(); return false; } // Cek Interrupt D
      setMotor(i, direction);
      delay(stepDelay);
    }
  }
  return true;
}

// Fungsi Delay yang bisa di-interrupt tombol D
bool holdSpeed(int duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)duration) {
    if (digitalRead(RF_D_PIN) == HIGH) {
      performSoftStop();
      return false; 
    }
    delay(10); // Cek setiap 10ms
  }
  return true;
}

// Fungsi Stop Bertahap (Dipanggil saat D ditekan)
void performSoftStop() {
  Serial.println(F("Status: SOFT STOP ACTIVATED"));
  
  // Hitung waktu ramp down agar proporsional. 
  // Kalau speed cuma 100, jangan nunggu 2.5 detik, tapi lebih cepat.
  int stopDuration = (int)((float)currentSpeed / (float)MAX_PWM * RAMP_TIME);
  
  // Kita manual ramp down di sini
  int startSpeed = currentSpeed;
  int stepDelay = 0;
  
  if (startSpeed > 0) {
      stepDelay = stopDuration / startSpeed;
  }
  if (stepDelay < 1) stepDelay = 1;

  for (int i = startSpeed; i >= 0; i--) {
    setMotor(i, currentDir);
    delay(stepDelay);
  }
  
  hardStop();
  isRunningSequence = false;
  Serial.println(F("Status: MOTOR STOPPED"));
}

// --- LOW LEVEL MOTOR CONTROL ---
void setMotor(int speed, int direction) {
  currentSpeed = speed; // Update global variable
  currentDir = direction;

  // Safety limit
  speed = constrain(speed, 0, 255);

  if (speed == 0) {
    hardStop();
    return;
  }

  if (direction == 1) { // Kanan / Maju
    analogWrite(LPWM_PIN, 0);
    digitalWrite(LPWM_PIN, LOW);
    analogWrite(RPWM_PIN, speed);
  } 
  else if (direction == -1) { // Kiri / Mundur
    analogWrite(RPWM_PIN, 0);
    digitalWrite(RPWM_PIN, LOW);
    analogWrite(LPWM_PIN, speed);
  }
}

void hardStop() {
  currentSpeed = 0;
  currentDir = 0;
  analogWrite(RPWM_PIN, 0);
  digitalWrite(RPWM_PIN, LOW);
  analogWrite(LPWM_PIN, 0);
  digitalWrite(LPWM_PIN, LOW);
}