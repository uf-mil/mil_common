#!/usr/bin/python


class ThrusterID:
    FHL = 0
    FHR = 1
    FVL = 2
    FVR = 3
    BHL = 4
    BHR = 5
    BVL = 6
    BVR = 7


class ThrusterKillBoardConstants:
    # Constants for kill/go:
    KILL_MESSAGE = 0x4B
    HARD_KILL = 0x48
    SOFT_KILL = 0x53
    COMMAND = 0x43
    RESPONSE = 0x52
    GO = 0x47
    ASSERTED = 0x41
    UNASSERTED = 0x55
    END = 0x00
    # Constants for thruster commands:
    THRUSTER_MESSAGE = 0x54
