import pyttsx3

def speak_text(text):
    # Initialize the text-to-speech engine
    engine = pyttsx3.init()

    # Set properties before adding anything to the speaking queue
    engine.setProperty('rate', 150)  # Speed percent (can go over 100)
    engine.setProperty('volume', 1)  # Volume 0-1

    # Add text to the queue
    engine.say(text)

    # Flush and wait for the speaking to finish
    engine.runAndWait()


