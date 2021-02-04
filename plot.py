import os, sys, json, datetime
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy

x=[]
y=[]
setpoint=[]

# plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
lineActualTemp, = ax.plot(x, y, 'b-')
lineSetpoint, = ax.plot(x, setpoint, 'r-')
ax.grid()

#MQTT function for receiving messages periodically
def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    xNew = int(data["Uptime"])
    yNew = float(data["ActualTemp"])
    setpointNew = float(data["Setpoint"])

    lineActualTemp.set_xdata(numpy.append(lineActualTemp.get_xdata(), xNew))
    lineActualTemp.set_ydata(numpy.append(lineActualTemp.get_ydata(), yNew))

    lineSetpoint.set_xdata(numpy.append(lineSetpoint.get_xdata(), xNew))
    lineSetpoint.set_ydata(numpy.append(lineSetpoint.get_ydata(), setpointNew))

    ax.set_xlim(min(lineActualTemp.get_xdata()), max(lineActualTemp.get_xdata()))
    ax.set_ylim(min(lineActualTemp.get_ydata()), max(lineActualTemp.get_ydata()))
    plt.draw()

# MQTT variables, set the host IP
host = "mqtt.host.ip.here"
port = 1883
topic = "espresso/status"

client = mqtt.Client("PLOTTING")
client.connect(host, port)
print("Subscribing to topic", topic)
client.subscribe(topic)
client.on_message = on_message
client.loop_start()

plt.show()

client.loop_stop()