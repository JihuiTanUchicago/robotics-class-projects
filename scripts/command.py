import requests
import time
import subprocess

# Store the current running process
current_process = None
def get_force_signal():
    url = "http://ec2-35-94-176-147.us-west-2.compute.amazonaws.com:5050/get-data"
    try:
        response = requests.get(url)
        response.raise_for_status()  # Raise an exception for HTTP errors
        data = response.json()
        print(data)
        return data.get("params_dict", {}).get("force_signal", None)
    except requests.RequestException as e:
        print(f"HTTP Request failed: {e}")
        return None
    except ValueError:
        print("Failed to parse JSON response")
        return None

def run_command(command):
    global current_process
    # Terminate the current process if it is running
    if current_process and current_process.poll() is None:
        current_process.terminate()
        current_process.wait()
    try:
        # Start a new process
        current_process = subprocess.Popen(command, shell=True)
    except subprocess.CalledProcessError as e:
        print(f"Command '{command}' failed with error: {e}")

def main():
    prev_action = None
    while True:
        force_signal = get_force_signal()
        # force_signal = input()
        if force_signal != prev_action:
            if force_signal == "SINGLE_TAP":
                print("SINGLE_TAP(drive_square) command going to run...")
                run_command("rosrun warmup_project drive_square.py")
            elif force_signal == "DOUBLE_TAP":
                print("DOUBLE_TAP(follow_person) command going to run...")
                run_command("rosrun warmup_project follow_person.py")
            elif force_signal == "LONG_PRESS":
                print("LONG PRESS(stop_robot) command going to run...")
                run_command("rosrun warmup_project stop_robot.py")
            prev_action = force_signal
        time.sleep(1)

if __name__ == "__main__":
    main()
