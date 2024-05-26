#!/usr/bin/env python3

import rospy
from robocourier.msg import RobotAction
from std_msgs.msg import Empty
from openai import OpenAI
import json
import record_voice
import speak

# removed api key handler in order to push
client = OpenAI(api_key = "API key")
tools = [
            {
                "type": "function",
                "function": {
                    "name": "get_drink_type",
                    "description": "Output the drink that the user asks for, if any. It could only be one of pepsi, coke, or sprite",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "drink_type": {
                                "type": "string",
                                "description": "A keyword that specifies the drink type that the user wants. The keyword could only be \"pepsi\", \"coke\", or \"sprite\"",
                            }
                        },
                        "required": ["drink_type"]
                    },
                },
            }
        ]
messages = [{"role": "system", "content": "you are a super hilarious comedian, but keep your response short and concise(less than 10 words)!! Make jokes all the time(except when function calling, where you have to be precise).You start introducing yourself as Robo Courrier, who is the fastest drink delievery robot. Pay close attention to what drinks they want and invoke function calling appropriately."}]

def to_index(node_name):
    if len(node_name) == 2:
        return 26 + ord(node_name.lower()[1])-97
    elif len(node_name) == 1:
        return ord(node_name.lower())-97

class ActionManager(object):

    def __init__(self):
        # initialize this node
        rospy.init_node('action_manager')

        # list of actions to take
        # may get updated with additional node for receiver
        self.actions = {
            "pepsi": 1,
            "coke": 2,
            "sprite": 3
        }
        self.action_idx = 0

        # establish /robocourier/state subscriber for receiving pings from robot
        rospy.Subscriber("/robocourier/state", Empty, self.determine_action)

        # establish /robocourier/robot_action publisher for sending actions to robot
        self.action_pub = rospy.Publisher("/robocourier/robot_action", RobotAction, queue_size=10)
        print("action manager initialized")

        self.run()

    # callback function for receiving a ping from robot
    def determine_action(self, data):
        has_new_task = False
        print("How's it going? Wanna drink? Or what?")
        speak.speak_text("How's it going? Wanna drink? Or what?")
        while has_new_task == False and not rospy.is_shutdown():
            # check if index isn't out of range
            
            record_voice.record_voice()
            audio_file= open("output.mp3", "rb")
            translation = client.audio.translations.create(
                model="whisper-1", 
                file=audio_file
            )
            audio_file.close()
            user_message = translation.text
            messages.append({"role": "user", "content": user_message})
            print(f"user_message is: {user_message}")
            completion = client.chat.completions.create(
                    model="gpt-4o",
                    messages=messages,
                    tools=tools,
                    tool_choice="auto"
            )
            response_message = completion.choices[0].message
            tool_calls = response_message.tool_calls
            if tool_calls:
                drink_type = json.loads(tool_calls[0].function.arguments).get("drink_type")
                print(drink_type)
                speak.speak_text(f"{drink_type}? Sure thing! One moment please!")
                messages.append({"role":"assistant","content": f"{drink_type}? Sure thing! One moment please!"})
                msg = RobotAction(obj= drink_type, tag=self.actions[drink_type])
                self.action_pub.publish(msg)
                has_new_task = True
                print(f"msg with obj={drink_type} and tag={self.actions[drink_type]} published, the robot should move")
            else:
                print(completion.choices[0].message.content)
                messages.append({"role":"assistant", "content":completion.choices[0].message.content})
                speak.speak_text(completion.choices[0].message.content)
            """
            if self.action_idx < 3:
                # create action message
                current_action = self.actions[self.action_idx]
                msg = RobotAction(obj=current_action["object"], tag=current_action["tag"])
                self.action_pub.publish(msg)

                self.action_idx += 1
            # otherwise, send an empty message
            else:
                self.action_pub.publish(RobotAction())
            """
    def run(self):
        rospy.spin()

if __name__=="__main__":

    node = ActionManager()
