import serial

ser = serial.Serial('/dev/ttyACM1', 9600)
print("CTRL + C pour arr�ter")

while True:
    led = int(input('Quelle LED ? (Rouge=1 / Vert=2 / Bleu=3) : '))
    action = int(input("Allumer=1 / Eteindre=0 : "))

    if action == 1:
        ser.write(str(led).encode())
    else:
        msg = led + 3
        ser.write(str(msg).encode())