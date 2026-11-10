# Start here {#start}

This guide teaches **dora** from zero, using the two examples that the dora project itself ships, copied unchanged into this repository. Every file is shown in full. Every line that matters is explained. Every command shows the output we actually got when we ran it on a MacBook with dora 1.1.0.

You do not need to know ROS, robotics or networking. You need to be able to open a terminal and read a little Python.

## What problem does dora solve? {#problem}

A robot is never one program. Even a simple one runs several programs at the same time:

- one that reads the **camera**
- one that **recognises objects** in the camera image
- one that **shows** the result on a screen
- later: one that **plans** a motion, one that **drives the motors**

These programs have to pass data to each other, many times per second: images, positions, commands. Writing that plumbing yourself is hard. Which program starts first? What happens when one crashes? How does a 1 MB camera image get to three other programs without copying it three times?

**dora is that plumbing.** You write each program as a small, separate Python file. You write one text file that says which program sends what to which other program. dora starts all the programs, delivers the data between them, and restarts a program if it crashes.

## The picture to keep in mind {#picture}

<div class="graph">
  <div class="box">camera</div><div class="arrow">image →</div><div class="box">object_detection</div><div class="arrow">bbox →</div><div class="box">plot</div>
</div>

Each box is one Python program. dora calls it a **node**. Each arrow is a stream of messages. The whole drawing is called a **dataflow**, and you describe it in a file called `dataflow.yml`. That is the entire idea. Everything else in this guide is detail.

## What you will do, in order {#plan}

| Step | What you do | Folder |
| --- | --- | --- |
| 1 | Install the tools | |
| 2 | Learn six words | |
| 3 | Run the official first example and read every line | `official/python-dataflow` |
| 4 | Change it: add your own node | `steps/00-first-change` |
| 5 | Run the official camera + AI example | `official/python-yolo-detection` |
| 6 | No webcam? Swap one node for a fake camera | `steps/01-fake-camera` |
| 7 | Make it survive crashes and slow parts | `steps/02-robust` |
| 8 | Look inside a running dataflow | `steps/01-fake-camera` |
| 9 | Record and replay | `steps/04-record-replay` |
| 10 | More examples and two robot simulations | `extras/` |

# 1 · Install {#install}

You install three things. It helps to know which is which, because people mix them up.

| What | What it is | How it gets installed |
| --- | --- | --- |
| **uv** | A tool that installs Python and Python packages, quickly and in a separate folder per project | You install it once, see below |
| **the dora command** | A program you type in the terminal: `dora run`, `dora start`, `dora list`. It starts and supervises your nodes | `./run.sh` installs it the first time |
| **the dora Python library** | What your node code imports: `from dora import Node`. The package is called `dora-rs` | `./run.sh` installs it the first time |

**Careful:** `pip install dora` installs a completely different package that has nothing to do with this. The correct package is `dora-rs`. The import is still `dora`.

## Step by step {#install-steps}

**1. Install uv** (skip if `uv --version` already prints a version):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**2. Get the project:**

```bash
git clone https://github.com/tchavtchavadzetornike/Robotics-101.git
cd Robotics-101
```

**3. Run the launcher once with no arguments.** The first time, it installs the dora command (version 1.1.0) and shows what can be run:

```text
$ ./run.sh
installing the dora command line tool (dora-rs-cli 1.1.0)
usage: ./run.sh <name> [--stop-after 20s] [--raw]

  the lessons, in order:
    first          official example: sender -> transformer -> receiver
    first-change   the same graph plus one new node you wrote
    yolo           official example: webcam -> YOLO -> window (needs a webcam)
    fake-camera    the YOLO example with a fake camera, no webcam needed
    robust         fake-camera plus restarts and dropping old frames
```

**4. Check the dora command works:**

```text
$ dora --version
dora-cli 1.1.0
```

That is all. The Python packages (the dora library, numpy, and later the AI model) are installed automatically the first time you run an example that needs them.

## What is in the project folder {#files}

```text
dora-robotics/
├── README.md                  short intro, points to this guide
├── run.sh                     the launcher: ./run.sh first, ./run.sh fake-camera ...
├── pyproject.toml             the list of Python packages the nodes need
├── uv.lock                    exact package versions, written by uv, do not edit
├── docs/                      this guide: guide.md and the built index.html
├── tools/pretty-logs.py       makes dora's log output readable, used by run.sh
├── official/                  the two official dora examples, unchanged
│   ├── python-dataflow/          sender → transformer → receiver
│   └── python-yolo-detection/    webcam → object_detection → plot
├── steps/                     the lessons, each one small change to an official example
│   ├── 00-first-change/          add a fourth node
│   ├── 01-fake-camera/           replace the webcam with a fake camera
│   ├── 02-robust/                restarts and dropping old frames
│   └── 04-record-replay/         record a run, play it back
└── extras/                    more examples and two robot simulations
```

Every example folder has the same shape: **one `dataflow.yml`** that describes the graph, and **one `.py` file per node**.

# 2 · Six words {#words}

These six words are all the vocabulary you need. Each comes with an example from the official camera demo.

| Word | Meaning | Example |
| --- | --- | --- |
| **node** | One program in the robot. Runs as its own process, so if it crashes, the others keep running | `object_detection.py` is a node |
| **dataflow** | The whole set of nodes and how they are connected. Written in `dataflow.yml` | the camera, detector and window together |
| **output** | A named stream that a node sends messages on | the camera sends on `image` |
| **input** | A named stream that a node receives. In the YAML you say which output it comes from | the detector's input `image` comes from `webcam/image` |
| **event** | One thing that happens to a node: a message arrived, or dora asks it to stop | a new camera frame arriving is an `INPUT` event |
| **Arrow** | The data format of every message. An efficient, standard way to store a list of numbers or a small table in memory | a camera frame is sent as one Arrow array of 921,600 numbers |

Two more words appear in log output. You do not have to manage them, but you will see them:

- **daemon:** the dora helper process that starts your nodes and delivers their messages. `dora run` starts one for you
- **coordinator:** a manager process that is only needed when a dataflow runs on several computers, or keeps running in the background. Section 8 uses it

# 3 · The first official example {#first}

Folder: `official/python-dataflow`. This is dora's own introductory example. Three nodes pass numbers to each other. No camera, no AI, nothing to install beyond the basics.

<div class="graph">
  <div class="box">sender</div><div class="arrow">message →</div><div class="box">transformer</div><div class="arrow">transformed →</div><div class="box">receiver</div>
</div>

The sender also sends `message` straight to the receiver, so the receiver gets two streams.

## Run it first {#first-run}

```bash
./run.sh first
```

Stop it with **Ctrl-C** (or add `--stop-after 10s`). This is what you see:

@out first-run

How to read one line, for example `15:22:59  receiver  INFO  Received message: [0]`:

- `15:22:59` is the time
- `receiver` is the node that wrote the line
- `INFO` is the log level
- the rest is the text the node logged with Python's `logging.info(...)`

The first five lines are dora itself: it builds, starts the three nodes, waits until all three say they are ready, and only then starts the flow of data.

`run.sh` passes dora's output through `tools/pretty-logs.py`, because dora prints each node log line as a long JSON record. To see the raw output, run `./run.sh first --raw`.

## The file that describes the graph {#first-yaml}

@file official/python-dataflow/dataflow.yml

| Line | What it says | In plain words |
| --- | --- | --- |
| 1 | `nodes:` | Here comes the list of programs. Every dataflow file starts like this |
| 2 | `- id: sender` | A new node called **sender**. The dash starts a new list item. The id is the name other nodes use to refer to it |
| 3 | `path: sender.py` | The program to run for this node: the file `sender.py` in the same folder |
| 4-5 | `outputs: - message` | The sender is allowed to send on one stream, called **message** |
| 7-8 | `- id: transformer`, `path: transformer.py` | A second node |
| 9-10 | `inputs: message: sender/message` | The transformer receives a stream it will call **message**. It comes from **sender**'s output **message**. Always written as *name in my code*: *node*/*output* |
| 11-12 | `outputs: - transformed` | The transformer sends on **transformed** |
| 16-18 | two inputs | The receiver gets **two** streams: the sender's `message` directly, and the transformer's `transformed` |

Notice what is **not** in this file: no network addresses, no start order, no message type definitions. Nodes do not know about each other. They only know the names of their own inputs and outputs.

**Check a dataflow file without running it:**

@out validate-ok

If you make a typo, for example `sendr/message`, dora tells you exactly where:

@out validate-typo

## The sender node {#first-sender}

@file official/python-dataflow/sender.py

| Line | What it does |
| --- | --- |
| 6 | `import pyarrow as pa`: pyarrow is the Python version of Arrow. Every message you send is a pyarrow array |
| 7 | `from dora import Node`: the dora library (package `dora-rs`) |
| 11 | `node = Node()` connects this program to dora. The node finds out its own id, inputs and outputs from dora, so the Python file needs no configuration |
| 15 | Send 100 messages, then stop |
| 16 | `node.try_recv()` checks if dora has an event for us, **without waiting**. The sender has no inputs, so the only event it can get is STOP |
| 18-21 | If dora says **STOP** (because you pressed Ctrl-C, or `--stop-after` ran out), leave the loop so the program ends cleanly |
| 26 | `pa.array([sent])`: a list with one number in it, as an Arrow array. For message 3 that is `[3]` |
| 27 | `node.send_output("message", data)`: send it on the output called **message**. The name must match line 5 of `dataflow.yml` |
| 28 | Write "Sent message 3" to the log. This is what appears as `sender INFO Sent message 3` |
| 32 | Wait 0.1 s, so 10 messages per second |

## The transformer node {#first-transformer}

@file official/python-dataflow/transformer.py

| Line | What it does |
| --- | --- |
| 10 | Connect to dora, as before |
| 12 | `for event in node:` **This loop is the heart of almost every node.** dora hands the node one event at a time. When there is nothing to do, the loop simply waits |
| 13 | `INPUT` means a message arrived on one of the node's inputs |
| 15 | `event["value"]` is the message itself, an Arrow array. `.to_pylist()` turns it into a normal Python list, for example `[3]` |
| 16 | Take the number out of the list: `3` |
| 19-26 | Build a small table with one row and three columns: `doubled` = 6, `description` = "Message #3", `is_even` = False. In Arrow this is called a **struct array** |
| 28 | Send the table on the output **transformed** |
| 33-35 | On STOP, leave the loop |

A struct array is how you send structured data in dora. There is no separate message definition file: the column names travel with the data.

## The receiver node {#first-receiver}

@file official/python-dataflow/receiver.py

| Line | What it does |
| --- | --- |
| 13 | `event["id"]` says **which input** the message arrived on: `"message"` or `"transformed"`. These are the names from lines 17 and 18 of `dataflow.yml` |
| 14-17 | A message from the sender: print the list, for example `[3]` |
| 18-21 | A table from the transformer: `.to_pylist()` turns the one-row table into `[{'doubled': 6, 'description': 'Message #3', 'is_even': False}]` |
| 24-26 | On STOP, leave the loop |

## What happened, step by step {#first-timeline}

When you typed `./run.sh first`, this is the sequence:

1. `run.sh` went into `official/python-dataflow` and ran `dora run dataflow.yml --uv`. `--uv` means "start each Python node with uv, inside this project's Python environment"
2. dora read `dataflow.yml` and found three nodes
3. dora started three separate programs: `sender.py`, `transformer.py`, `receiver.py`
4. Each program called `Node()`. That is the "node is ready" line in the log
5. When all three were ready, dora started the dataflow
6. The sender sent `[0]`. dora delivered it to **both** the transformer and the receiver, because both list `sender/message` as an input
7. The transformer turned `[0]` into a table and sent it. dora delivered the table to the receiver
8. Repeat ten times per second
9. On Ctrl-C, dora sent STOP to every node. Each left its loop and ended: "finished successfully"

# 4 · Your first change: add a node {#change}

Folder: `steps/00-first-change`. The task: add a fourth node, **checker**, that listens to the transformer and logs only the even numbers. Nothing else changes. The other three files are copies of the official ones.

## Step 1: write the new node {#change-node}

Create `checker.py` next to the other nodes:

@file steps/00-first-change/checker.py

| Line | What it does |
| --- | --- |
| 9 | Connect to dora |
| 10 | The event loop, same as every node |
| 11 | Only react to messages on the input called **transformed** |
| 12 | The message is a one-row table. `.to_pylist()[0]` gives that row as a Python dictionary |
| 13-14 | If the row says `is_even`, log it |
| 15-16 | Leave on STOP |

## Step 2: add it to the graph {#change-yaml}

Add one block at the end of `dataflow.yml`. The block marked NEW is the only difference to the official file:

@file steps/00-first-change/dataflow.yml

The new block says: there is a node called **checker**, run `checker.py`, and give it an input called **transformed** that comes from **transformer**'s output **transformed**. The transformer does not need to change at all. It now simply has two receivers instead of one.

## Step 3: check and run {#change-run}

```bash
cd steps/00-first-change && dora validate dataflow.yml && cd ../..
./run.sh first-change
```

@out first-change

Every new node you will ever write in dora follows these three steps: **write the Python file with the event loop, add one block to `dataflow.yml`, connect its inputs.**

**Try it yourself:**

- Make the sender faster: change `time.sleep(0.1)` to `time.sleep(0.02)` in `steps/00-first-change/sender.py`
- Make checker send its own output: add `outputs: [even]` to its YAML block and `node.send_output("even", pa.array([row["doubled"]]))` in the code, then connect it to the receiver

# 5 · The official camera + AI example {#yolo}

Folder: `official/python-yolo-detection`. This is the example most people mean when they say "the dora demo": a webcam, an object detector (YOLO, an AI model that finds people, cars, cups and 77 other kinds of object in an image), and a window that draws boxes around what was found.

<div class="graph">
  <div class="box">webcam</div><div class="arrow">image →</div><div class="box">object_detection</div><div class="arrow">bbox →</div><div class="box">plot</div>
</div>

The webcam's `image` also goes straight to `plot`, so the window shows every frame, even while the detector is still busy with an older one.

**Needs:** a webcam, and about 2 GB of download the first time (the AI libraries). On macOS, the first frame triggers a pop-up asking to allow camera access for your terminal. **No webcam?** Read this section to understand the files, then run section 6, which uses the same detector and window with a fake camera.

```bash
./run.sh yolo
```

A window called **frame** opens with the webcam image and green boxes. Press **q** in the window or Ctrl-C in the terminal to stop.

## The graph {#yolo-yaml}

@file official/python-yolo-detection/dataflow.yml

| Line | What it says | In plain words |
| --- | --- | --- |
| 4-5 | `inputs: tick: dora/timer/millis/50` | The webcam node has no real input, so it gets a **timer** from dora: a tick every 50 ms. Each tick, it grabs one frame. 50 ms is 20 frames per second |
| 8-9 | `env: CAMERA_INDEX: 0` | A setting handed to the program as an environment variable. 0 is the first camera. A second camera would be 1 |
| 13 | `restart_policy: on-failure` | If the detector crashes, dora starts it again. Without this line, a crashed node stays dead |
| 15 | `image: webcam/image` | The detector receives the camera images |
| 23-24 | two inputs | The window receives the images **and** the boxes |

## webcam.py: grabbing frames {#yolo-webcam}

@file official/python-yolo-detection/webcam.py

| Line | What it does |
| --- | --- |
| 15 | Read the camera number from the `env:` block of the YAML |
| 22-24 | Open the camera with OpenCV (a common image library) and ask for 640 × 480 pixels |
| 27-28 | On every timer tick (an INPUT event) |
| 29 | Grab one frame from the camera |
| 33-43 | If the camera fails more than 10 times in a row (for example the USB cable was pulled), close and reopen it |
| 47 | `frame.ravel()` flattens the image (480 rows × 640 columns × 3 colours) into one long row of 921,600 numbers. Arrow sends this flat row |
| 48-52 | **metadata**: a small dictionary sent along with the message. Here it carries the width, the height and `bgr8`, which means blue-green-red colour order, 8 bits per colour. The receiver needs these to rebuild the image from the flat row |

## object_detection.py: finding objects {#yolo-detect}

@file official/python-yolo-detection/object_detection.py

| Line | What it does |
| --- | --- |
| 16-17 | Load the YOLO model once, when the node starts. The file `yolov8n.pt` (6 MB) is downloaded automatically the first time |
| 24 | The message as a numpy array: still the flat row of 921,600 numbers |
| 25-27 | Read width and height from the metadata |
| 30-31 | Skip broken frames that do not have the right number of values |
| 33 | Rebuild the image: reshape the flat row back into 480 × 640 × 3 |
| 34 | OpenCV uses blue-green-red order, YOLO expects red-green-blue: reverse the colour order |
| 36 | Run YOLO on the image |
| 38-40 | For each object found: its box (x1, y1, x2, y2 in pixels), how sure the model is (0 to 1), and which kind of object it is (a number, 0 = person) |
| 43 | Put them side by side: 6 numbers per object |
| 47 | Send all boxes as one flat Arrow array on **bbox**. Three objects = 18 numbers. The metadata of the image is passed on unchanged |

## plot.py: drawing the result {#yolo-plot}

@file official/python-yolo-detection/plot.py

| Line | What it does |
| --- | --- |
| 21 | Remember the latest boxes. Starts empty |
| 25 | Which input did this message arrive on: `image` or `bbox`? |
| 28-32 | A new frame: rebuild the image from the flat row, using width and height from the metadata |
| 34-49 | Draw every remembered box as a green rectangle, with the object name and confidence. `LABELS` (from `utils.py`) turns 0 into "person", 2 into "car" and so on |
| 51 | Show the image in the window |
| 52-53 | If you press **q**, stop |
| 55-57 | New boxes arrived: reshape the flat row into rows of 6 numbers and remember them for the next frames |

This is why the window does not stutter even when the detector is slow: frames are drawn as soon as they arrive, with the most recent boxes available.

# 6 · No webcam? Replace one node {#fake}

Folder: `steps/01-fake-camera`. This lesson shows the biggest practical benefit of dora: **you can replace one node without touching the others**, as long as the new node sends the same kind of messages.

We replace `webcam.py` with `fake_camera.py`. Instead of a camera, it slowly moves across a photo of a street with a bus and people. The detector and the window are the **official files, unchanged**.

## The new node {#fake-node}

@file steps/01-fake-camera/fake_camera.py

| Line | What it does |
| --- | --- |
| 18 | Which photo to use. By default a test photo that comes with the YOLO library (`bus.jpg`). Set `PHOTO` in the YAML `env:` to use your own |
| 21-26 | Load the photo and enlarge it a bit, so there is room to move around in it |
| 29-34 | For step number *n*, cut out a 640 × 480 piece at a slowly changing position. This looks like a camera panning across the street |
| 41-43 | On every timer tick, cut out the next piece |
| 45-53 | Send it **exactly like webcam.py does**: same output name `image`, same flat array, same metadata `width`, `height`, `encoding: bgr8` |

Lines 45 to 53 are the contract. Because they match the official webcam, the detector and the window cannot tell the difference.

## The graph {#fake-yaml}

@file steps/01-fake-camera/dataflow.yml

Compared to the official file, two things changed:

- The first node runs `fake_camera.py` instead of `webcam.py`. Its id stays `webcam`, so the other nodes' inputs (`webcam/image`) stay exactly the same
- The other two nodes point at the official files with `../../official/python-yolo-detection/...`. Paths in `dataflow.yml` are relative to the folder the YAML file is in

## Run it {#fake-run}

```bash
./run.sh fake-camera
```

@out fake-camera-run

The first start takes about 40 seconds: the AI library is large and loads slowly, and the model file is downloaded once. Later starts are faster.

# 7 · Make it survive crashes and slow parts {#robust}

Folder: `steps/02-robust`. Two real problems showed up when we ran the fake camera example, and dora fixes both with a few lines of YAML.

## Problem 1: the detector is slower than the camera {#robust-slow}

The camera sends 20 frames per second. YOLO on a laptop CPU sometimes cannot keep up, especially right after starting. When we looked at the detector's log, dora was throwing frames away:

@out discarding

This is dora protecting the node: every input has a **queue**, a waiting line of messages, and by default it holds 10. When the waiting line is full, dora drops a message instead of letting memory grow forever.

For a camera, the right behaviour is to keep **only the newest** frame. A detector working through a backlog of old frames would draw boxes where objects used to be, seconds ago. So we say that explicitly.

## Problem 2: a crashed node {#robust-crash}

The official YAML already has `restart_policy: on-failure`. We add two lines so it does not restart forever, and with a growing pause.

## The changed graph {#robust-yaml}

@file steps/02-robust/dataflow.yml

| Line | What it says | In plain words |
| --- | --- | --- |
| 13 | `max_restarts: 5` | After 5 crashes, give up instead of crashing and restarting in an endless loop |
| 14 | `restart_delay: 1.0` | Wait 1 second before the first restart, then 2, then 4 and so on. Gives a broken camera or a full disk time to recover |
| 16-17 | `image:` then `source: webcam/image` | The **long form** of an input. Needed as soon as an input gets its own settings. `source` is what came after the colon in the short form |
| 18 | `queue_size: 1` | At most one frame waiting |
| 19 | `queue_policy: drop_oldest` | When a new frame comes in and one is already waiting, throw away the old one |

## Test the restart {#robust-test}

We started the robust dataflow in the background (how: section 8), killed the detector on purpose, and listed the nodes before and after:

@out restart

Look at the `PID` (process id) and `RESTARTS` columns: the detector has a **new** process id and a restart count of 1. The camera and the window kept their process ids: **they never noticed.** That is what "each node is its own process" buys you.

# 8 · Look inside a running dataflow {#inspect}

`dora run` is the simple way: everything in one terminal, Ctrl-C stops it. To look inside a dataflow while it runs, start it in the background instead. This needs two more commands, `dora up` and `dora start`.

| Command | What it does |
| --- | --- |
| `dora up` | Starts the coordinator and a daemon in the background. Do this once |
| `dora start dataflow.yml --name yolo --uv --debug --detach` | Starts the dataflow under the name **yolo**. `--detach` gives you the terminal back. `--debug` allows `topic echo` to read the messages |
| `dora list` | All running dataflows |
| `dora node list -d yolo` | The nodes of one dataflow: process id, CPU, memory, restarts |
| `dora logs yolo --node object_detection` | One node's log output |
| `dora topic echo -d yolo object_detection/bbox` | Print the messages on one output, live. Ctrl-C to stop |
| `dora stop --name yolo` | Stop the dataflow |
| `dora down` | Stop the coordinator and daemon |

Here is the real session, run in `steps/01-fake-camera`:

@out inspect

**How to read the `bbox` message.** It is the flat row of 6 numbers per object, sent by line 47 of `object_detection.py`. The first group of six:

| x1 | y1 | x2 | y2 | confidence | class |
| --- | --- | --- | --- | --- | --- |
| 133.48 | 0.0 | 261.98 | 287.73 | 0.8868 | 0.0 |

A box from pixel (133, 0) to (262, 288), the model is 89 % sure, and class 0 means **person**. The three groups in the first line are three people, the last one only 37 % sure. The timestamps in the metadata are 50 ms apart: the detector kept up with all 20 frames per second at that moment.

**One command that did not work for us:** `dora topic hz` (messages per second) reported 0 samples in every attempt with dora 1.1.0, even while `topic echo` showed messages arriving. Use `topic echo` and the timestamps instead.

# 9 · Record and replay {#record}

Folder: `steps/04-record-replay`. **Status: not yet run on this machine.**

The idea: `dora record` saves every message on chosen outputs into one file while the dataflow runs. `dora replay` plays that file back later **instead of** the original node. For example: record the camera once, then test a new detector on exactly the same frames as often as you like, without the camera.

The record and replay commands start two helper programs, `dora-record-node` and `dora-replay-node`. They are not included in the dora 1.1.0 download, and they are not on crates.io either, although dora's error message says `cargo install dora-record-node`. They have to be built from dora's source code with Rust:

```bash
# once: install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# once: build the two helpers from the dora 1.1.0 source
git clone --depth 1 --branch v1.1.0 https://github.com/dora-rs/dora.git /tmp/dora-src
cargo install --locked --path /tmp/dora-src/binaries/record-node
cargo install --locked --path /tmp/dora-src/binaries/replay-node
```

Then, from `steps/01-fake-camera`:

```bash
# record about 30 s of camera frames, then press Ctrl-C
dora record dataflow.yml --topics webcam/image -o run.drec

# play them back: the recording replaces the webcam node
dora replay run.drec --replace webcam --speed 1
```

| Option | Meaning |
| --- | --- |
| `--topics webcam/image` | Which outputs to save. Without it, all outputs are saved |
| `-o run.drec` | The file to write |
| `--replace webcam` | Do not start the webcam node, feed the recorded frames instead |
| `--speed 1` | Real time. 2 is twice as fast, 0 as fast as possible |
| `--loop` | Start again at the end |

# 10 · More examples {#extras}

The `extras/` folder has smaller examples for individual dora features and two robot simulations. They were all run on this machine with dora 1.1.0 except where noted.

| Run with | Folder | What it shows |
| --- | --- | --- |
| `./run.sh hello` | `extras/examples/01-hello-graph` | three nodes with a timer, the smallest graph |
| `./run.sh camera` | `extras/examples/02-camera-pipeline` | camera, detector, viewer with simple colour detection, shown in Rerun |
| `./run.sh types` | `extras/examples/03-types` | optional message types, `dora validate` catching a wrong connection |
| `./run.sh modules` | `extras/examples/04-modules` | a group of nodes in its own file, used like one node |
| `./run.sh services` | `extras/examples/05-services-actions` | question-and-answer messages, long jobs with progress and cancel |
| `./run.sh faults` | `extras/examples/06-fault-tolerance` | restarts, a node that stops sending, a node that is too slow |
| see README | `extras/examples/08-live-changes` | add and replace nodes while the dataflow keeps running |
| see README | `extras/examples/09-distributed` | one dataflow over two computers, simulated on one laptop |
| see README | `extras/examples/10-ros2-bridge` | talking to ROS 2. Not run here, needs ROS 2 |
| `./run.sh mobile-robot` | `extras/simulations/mobile-robot` | a wheeled robot with a simulated laser scanner drives to 5 goals around obstacles |
| `./run.sh arm` | `extras/simulations/arm` | a two-joint arm follows a moving target |

Each folder has its own README.

# 11 · Problems and fixes {#troubleshooting}

Every problem below happened to us while building this project.

| What you see | Why | Fix |
| --- | --- | --- |
| `ImportError: cannot import name 'Node' from 'dora'` | `pip install dora` installed the wrong package | `pip uninstall dora`, then use `dora-rs`. With this project, `./run.sh` does it right |
| A node log line is a long `{"timestamp":...,"fields":...}` record | dora prints Python log lines as JSON | `./run.sh` cleans it. By hand: `dora run dataflow.yml --uv 2>&1 \| python3 tools/pretty-logs.py` |
| `node exited before initializing dora connection` | The node took too long to start, for example while another uv install was running | Wait for the install to finish and run again |
| The camera example takes 40 s to start | The AI library is large, and the model is downloaded once | Normal. Later starts are faster |
| `Discarding event for input image due to queue size limit` | A node is slower than its input | Section 7: `queue_size: 1`, `queue_policy: drop_oldest` |
| `source node sendr ... does not exist` | Typo in `dataflow.yml` | `dora validate dataflow.yml` before running |
| `dora topic echo` says debug inspection is required | The dataflow was started without `--debug` | `dora start ... --debug` |
| `no dataflow with name yolo is running` | It already stopped, or was started with `dora run` | `dora list`, and use `dora up` + `dora start` for background runs |
| `could not find dora-record-node` | Not part of the 1.1.0 download | Section 9 |
| No window appears on macOS | Camera permission denied for the terminal | System Settings → Privacy & Security → Camera |

# 12 · Cheat sheet {#cheatsheet}

| I want to | Command |
| --- | --- |
| run a dataflow in this terminal | `dora run dataflow.yml --uv` |
| run it for 20 seconds only | `dora run dataflow.yml --uv --stop-after 20s` |
| check a dataflow file | `dora validate dataflow.yml` |
| run it in the background | `dora up`, then `dora start dataflow.yml --name NAME --uv --detach` |
| see what runs | `dora list`, `dora node list -d NAME` |
| read one node's log | `dora logs NAME --node NODE` |
| see messages on an output | start with `--debug`, then `dora topic echo -d NAME NODE/OUTPUT` |
| stop | `dora stop --name NAME`, then `dora down` |

**A new node, every time:** write a `.py` file with `node = Node()` and `for event in node:`, add a block to `dataflow.yml` with `id`, `path`, `inputs` and `outputs`, run `dora validate`.

**Credits.** The files in `official/` are copied unchanged from the dora repository, release v1.1.0, `examples/python-dataflow` and `examples/python-yolo-detection`, Apache License 2.0, © the dora-rs contributors. Source: github.com/dora-rs/dora.
