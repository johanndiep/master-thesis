import serial
import re


nranges = 0
nsuccess = 0
j = 0

current_range = 0
last_pass = False

csv = open("node_otput.csv", "w")
csv.write("range, state")

meas = [0]*100

for i in range(0, 10):
    try:
        serial_port = serial.Serial("/dev/ttyACM" + str(i), 115200)
        print("Connected to serial /dev/ttyACM" + str(i))
        break

    except:
        pass

with serial_port as ser:
    while j < 1000:
        line = ser.readline().strip().decode("utf8")

        match = re.match("^distance 1: *([0-9]+)mm", line)

        if line == "Interrogating anchor 1":
            # Printing state
            template = "\r\033[2K{} Ranges {} Failed ({:.0f}% pass), Mean: {}m"
            
            percent_pass = 0
            
            if nsuccess > 0:
                percent_pass = 100*(nsuccess/nranges)
            
            distance = sum(meas) / len(meas)
            
            print(template.format(nranges, nranges-nsuccess, percent_pass, distance), end='')

            if nsuccess > 0:
                csv.write("{}, {}\n".format(current_range, 1 if last_pass else 0))

            nranges += 1            
            last_pass = False

        elif match:
            current_range = int(match.group(1)) / 1000.0

            meas = [int(match.group(1)) / 1000.0] + meas
            meas.pop()

            nsuccess += 1
            last_pass = True

    j += 1

print("Distance value: ", distance)
