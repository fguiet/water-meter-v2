# water-meter version 2 (LoRaWAN enabled)

## Forewords

This project aims at getting an analog signal from a water meter and use it for a house automatization system to control the water consumption.

## Specifications

This project uses :

* CNY70 reflective sensor (that includes an infrared emitter and phototransistor)
* Arduino Pro Mini 3.3v, 8Mhz
* Solar Panel
* TP4056 (Lithium Battery Charger Module)
* RFM95W LoRa Module

## Datasheet

* Schema

Available here (locate Water Meter v2 Datasheet) : <https://easyeda.com/fguiet/water-meter>

## Sensor accuracy

Must be done on this new project...but first tests are ok

## 3D CNY70 Holder

See my old project, nothing changed here : <https://github.com/fguiet/water-meter>

## In real life

![Water Meter V2](images/water-meter-v2-lorawan.png)

## Issue

At the moment, one issue : <https://github.com/matthijskooijman/arduino-lmic/issues/293>
