import serial

ser = serial.Serial(a
    port='/dev/serial0', 
    baudrate=9600,     
    timeout=1             
)

print("Listening for incoming data...")

try:
    while True:
        if ser.in_waiting > 0:
            received_data = ser.read().decode('utf-8')
            print(f"Received: {received_data}")

except KeyboardInterrupt:
    print("Exiting")
finally:
    ser.close()
