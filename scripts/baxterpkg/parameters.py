#!/usr/bin/env python

import json
import os


class BaxterParamsServer(object):

    _SCRIPTS_FOLDER_PATH = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, os.pardir))
    _PARAMETERS_FOLDER_PATH = os.path.join(_SCRIPTS_FOLDER_PATH, "params")

    def __init__(self, file_name):

        self.file = os.path.join(BaxterParamsServer._PARAMETERS_FOLDER_PATH, file_name)

        with open(self.file) as json_file:
            self.data = json.load(json_file)

    def getParam(self, name):
        return self.data[name]
