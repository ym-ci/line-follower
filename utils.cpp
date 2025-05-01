#include <Arduino.h>
#include "utils.h"

void printVar(const char* name, int value) {
    Serial.print(name);
    Serial.print(":");
    Serial.println(value);
}
void printVar(const char* name, float value) {
    Serial.print(name);
    Serial.print(":");
    Serial.println(value);
}
void printVar(const char* name, const char* value) {
    Serial.print(name);
    Serial.print(":");
    Serial.println(value);
}
void println(int i) {
    Serial.println(i);
}
void println(const char* s) {
    Serial.println(s);
}
void print(int i) {
    Serial.print(i);
}
void print(const char* s) {
    Serial.print(s);
}
void println() {
    Serial.println();
}
