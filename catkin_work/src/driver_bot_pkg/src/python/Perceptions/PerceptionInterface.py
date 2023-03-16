#!/usr/bin/env python3
from abc import ABC, abstractmethod


class PerceptionInterface(ABC):
    """PerceptionInterface shows common functions for AngleDetection, DistanceDetection, ObjectDetection"""
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def dataAnalyses(self):
        pass

    @abstractmethod
    def detection(self):
        pass
