import paho.mqtt.client as mqtt

# Define callback function to handle messages
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # Subscribe to topics once connected (e.g., all Fortress topics)
    client.subscribe("fortress/+/data")

def on_message(client, userdata, msg):
    # This function will be called whenever a message is received on the subscribed topic
    print(f"Received message on {msg.topic}: {msg.payload.decode()}")

# MQTT broker details
broker = "10.0.0.204"
port = 44925
username = ""
password = ""

# Create MQTT client instance
client = mqtt.Client()

# # Set username and password for authentication (if required)
# client.username_pw_set(username, password)

# Attach callback functions
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(broker, port, 60)

# Start the loop to process incoming messages
client.loop_start()

# Keep the program running
try:
    while True:
        pass  # The loop will handle incoming messages asynchronously
except KeyboardInterrupt:
    print("Disconnected from broker")
    client.loop_stop()
    client.disconnect()