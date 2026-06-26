import serial
import csv

ser = serial.Serial('/dev/ttyUSB0', 115200)

with open('imu_data.csv', 'w', newline='') as f:
    writer = csv.writer(f)

    writer.writerow([
        'timestamp',
        'ax','ay','az'
    ])

    while True:
        line = ser.readline().decode().strip()

        if line:
            print(line)

            fields = line.split(',')

            if len(fields) == 4:
                writer.writerow(fields)
                f.flush()