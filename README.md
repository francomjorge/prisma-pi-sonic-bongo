# SONIC BONGO — ESP32-S3 N16R8 Diagnostic

Herramienta de diagnóstico para una placa ESP32-S3 N16R8 con layout compatible ESP32-S3-DevKitC-1 de 44 pines.

## Objetivo

Levantar un Access Point local y servir una web en `http://192.168.4.1` que represente gráficamente la placa y muestre el nivel lógico de los GPIO seguros en tiempo real.

## Baseline

- ESP32-S3 / Arduino framework
- AP abierto: `PRISMA-S3-DIAG`
- Sin contraseña Wi-Fi
- Web local: `192.168.4.1`
- Monitoreo no destructivo: no se manejan salidas HIGH/LOW
- GPIO sensibles o reservados se muestran, pero el firmware no los toca
- N16R8: 16 MB Flash + 8 MB Octal PSRAM esperados; el firmware muestra los valores detectados realmente por el chip

## Seguridad de la primera prueba

Para la primera prueba, alimentar el ESP32 únicamente por USB. No conectar todavía fuentes externas de 5 V o 3.3 V.

## Firmware

`firmware/ESP32S3_N16R8_PinMonitor/ESP32S3_N16R8_PinMonitor.ino`
