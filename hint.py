#!/usr/bin/env python3
"""Hints for the STUDENT blocks, one at a time, from the idea to the code.

    python3 hint.py          which hints you have opened so far
    python3 hint.py 1.3      the next hint for STUDENT 1.3
    python3 hint.py 1.3 -r   the hints of 1.3 you already have, again
"""
import sys

from ohm_icp import hints

if __name__ == "__main__":
    sys.exit(hints.main())
