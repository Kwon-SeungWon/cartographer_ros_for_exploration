#!/usr/bin/env python3

"""
Entry point for waypoint_node_manager.
This file imports the compiled Cython module and runs the main function.
"""

from .waypoint_node_manager import main

if __name__ == "__main__":
    main() 