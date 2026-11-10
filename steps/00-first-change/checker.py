"""New node: reads the transformer's output and logs only the even messages."""

import logging

from dora import Node


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "transformed":
            row = event["value"].to_pylist()[0]
            if row["is_even"]:
                logging.info("%s is even, doubled it is %d", row["description"], row["doubled"])
        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
