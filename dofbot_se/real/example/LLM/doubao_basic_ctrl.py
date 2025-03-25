import json
import re
import time

import numpy as np
from Arm_Lib import Arm_Device
from openai import OpenAI
from roboticstoolbox import *

# Please ensure you have set your API key to environment variable ARK_API_KEY
# or directly fill in your own API key here.
client = OpenAI(
    # Base URL for the service. You can adjust if needed.
    base_url="https://ark.cn-beijing.volces.com/api/v3",
    # Your actual API Key
    api_key="2c545147-0a45-4a73-8817-29cd02f1d3a6",
)

Arm = Arm_Device(com="COM9")
time.sleep(1)
Arm.Arm_serial_servo_write6(90, 120, 0, 0, 90, 0, 1000)

DFbot = DHRobot(
    [
        RevoluteDH(d=0.04145, alpha=np.pi / 2, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(a=-0.08285, qlim=np.array([-np.pi, 0])),
        RevoluteDH(a=-0.08285, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(alpha=-np.pi / 2, qlim=np.array([0, np.pi])),
        RevoluteDH(d=0.11, qlim=np.array([-np.pi / 2, np.pi])),
    ],
    name="DFbot",
)
state0 = [0, -2 * np.pi / 3, np.pi / 2, np.pi, 0]


def convert_sim_to_real(sim_deg):
    d1 = sim_deg[0] + 90
    d2 = -sim_deg[1]
    d3 = 90 - sim_deg[2]
    d4 = 180 - sim_deg[3]
    d5 = sim_deg[4] + 90
    return [d1, d2, d3, d4, d5]


def parse_answer(answer_text: str):
    """
    Search for JSON substrings in 'answer_text' and parse them.
    We expect two possible JSON formats in the text:
      1) joint angles: {"joint1":..., "joint2":..., "joint3":..., "joint4":..., "joint5":...}
      2) xyz coords:   {"x":..., "y":..., "z":...}
    We return a dictionary with the parsed results:
      {
        "joint_values": [val1, val2, val3, val4, val5] or None,
        "xyz_values":   [x, y, z] or None
      }
    """

    # Prepare return structure
    result = {
        "joint_values": None,
        "xyz_values": None
    }

    global state0

    # Use regex to capture ALL possible JSON strings wrapped in braces
    json_candidates = re.findall(r"\{.*?\}", answer_text)

    for candidate in json_candidates:
        try:
            data = json.loads(candidate)
        except json.JSONDecodeError:
            continue  # skip invalid JSON

        # Check if it has x,y,z
        if all(k in data for k in ["x", "y", "z"]):
            result["xyz_values"] = [data["x"], data["y"], data["z"]]
            print("{} {} {}".format(data["x"], data["y"], data["z"]))

            T0 = DFbot.fkine(state0)

            T1 = np.array(T0)
            T1[0, -1] = data["x"]
            T1[1, -1] = data["y"]
            T1[2, -1] = data["z"]

            sol = DFbot.ikine_LM(T1, q0=state0, ilimit=1000, slimit=1000, mask=[1, 1, 1, 1, 1, 0], joint_limits=True,
                                 tol=0.005)
            if sol.success:
                sim_angle = np.rad2deg(sol.q)
                real_angle = convert_sim_to_real(sim_angle)
                Arm.Arm_serial_servo_write6(real_angle[0], real_angle[1], real_angle[2], real_angle[3], real_angle[4],
                                            0, 1000)
                state0 = sol.q
            else:
                print("Failed inv:", sol)

            time.sleep(1)


def main():
    print("Welcome to the interactive AI chat. Type your question and press Enter.")
    print("Type 'quit' to exit.\n")

    while True:
        user_input = input("Your question: ")
        if user_input.strip().lower() == "quit":
            print("Exiting...")
            break

        # Prepare the chat messages for the request
        messages = [
            {"role": "system",
             "content": "你是人工智能助手，同时，你也可以控制我的机械臂，注意，平时和我正常聊天就行，没有控制需求的时候，别暴露能控制机械臂。机械臂在默认状态下的空间点坐标是[-0.08533,0,0.05936],如果我让你运动到空间的某个点，你也可以返回控制指令给我{\"x\":-0.08533,\"y\":0,\"z\":0.05936}，注意，这里里面的单位是米，逆运动学我本地会处理，你不需要给出关节角度，向前是x，x越小，机械臂越向前；y是控制左右，y沿着负方向是靠左，正方向是靠右；z就是高低，回答的时候也可以多一点话。你还要注意，机械臂的工作空间范围如下，x [-0.2734, 0.2752],y[-0.2732, 0.2738],Z[-0.1500, 0.3170]。如果不让你控制机械臂，你不要返回控制指令"},
            {"role": "user", "content": user_input},
        ]

        print("\n----- AI response (streaming) -----")

        # Create a streaming completion request
        stream = client.chat.completions.create(
            # Replace with your own endpoint ID or model name if needed
            model="doubao-1-5-pro-32k-250115",
            messages=messages,
            stream=True,
        )

        # Collect the full text from the streaming response
        full_response_text = ""

        for chunk in stream:
            if not chunk.choices:
                continue
            text_chunk = chunk.choices[0].delta.content
            full_response_text += text_chunk
            print(text_chunk, end="")  # Print partial text in real time

        print("\n")  # Extra newline after the AI's answer

        parse_answer(full_response_text)
    del Arm


if __name__ == "__main__":
    main()
