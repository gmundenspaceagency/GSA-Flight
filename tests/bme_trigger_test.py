from time import sleep


altitudes = [455.07,454.9,454.9,455.07,455.16,455.42,455.51,455.86,456.21,456.38,456.73,457.25,457.6,458.21,458.82,459.43,460.38,460.99,462.04,462.91,463.6,464.39,465.43,466.22,468.92,469.18,469.26,469.09,469.09,469,468.83,468.92,469,46,46,468.9,468.9,468.9,468.9,469.2,469.0,469.0,469.2,469.1,46,469.0,468.9,468.9,468.8,468.8,468.8,468.83,468.92,468.83,469.44,469.18,469.09,469,468.92,468.92,468.92,471.35]

start_altitude = altitudes[0]
ascending = False
for i in range(1,len(altitudes)):
    print(altitudes[i])
    if altitudes[i] > start_altitude + 10:
        ascending = True
        print("ascending", i)
        start_altitude = altitudes[i]
        #calculate average speed of the last 5 altitudes
        if i > 5:
            average_speed = sum(altitudes[i-5:i]) / 5
            if average_speed < -5:
                print("descending", i)

    
    sleep(0.1)