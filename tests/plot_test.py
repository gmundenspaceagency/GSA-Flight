import matplotlib.pyplot as plt
import random
import time

def generate_random_number():
    return random.randint(0, 100)

def main():
    plt.ion()  # Interaktiver Modus für fortlaufendes Plotten

    fig, ax = plt.subplots()
    ax.set_xlabel('Zeit (Sekunden)')
    ax.set_ylabel('Zufallszahl')
    ax.set_title('Erhaltene Daten')

    line, = ax.plot([], [])

    plt.show()

    x_data = []
    y_data = []

    while True:
        x_data.append(len(x_data))
        y_data.append(float(input("Zahl eingeben:")))

        line.set_xdata(x_data)
        line.set_ydata(y_data)

        ax.relim()
        ax.autoscale_view()

        plt.draw()
        plt.pause(1)  # Pause für 1 Sekunde, um die Zufallszahl pro Sekunde zu aktualisieren

if __name__ == "__main__":
    main()
