# mission_interface

**The keyboard.** This is where a human types an instruction in plain English.
It's the very first step of the pipeline.

## Its role in the pipeline
It takes what you type and broadcasts it so the rest of the system can react.
It does nothing clever — it just gets your words onto the ROS "message bus."

## Input / output
- **Input:** text you type on the keyboard (standard input), inside its own
  terminal window.
- **Output:** publishes that text on the ROS topic **`/mission/prompt`**.
- **Who uses the output:** `mission_llm_planner` listens on `/mission/prompt`.

## The file: `mission_interface/prompt_publisher.py`

A small ROS node called `prompt_publisher`.

- `__init__` — sets up a **publisher** on `/mission/prompt`. A publisher is just
  a sender: whatever it publishes, any subscriber to that topic receives.
- `run()` — the input loop. It prints a prompt (`mission>`), waits for you to
  type a line, and on Enter wraps your text in a ROS `String` message and
  publishes it. Typing `quit`/`exit` or pressing Ctrl-D stops it.
- `main()` — standard ROS start-up/shut-down wrapper.

## Why it runs in its own terminal (xterm)
The main launch file starts this node inside a dedicated terminal window titled
**"1 INTERFACE"** so you actually have a place to type. The other pipeline nodes
each get their own window too, so you can watch the message flow stage by stage.
