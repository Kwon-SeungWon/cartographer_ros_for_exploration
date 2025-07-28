#!/usr/bin/env python3

import os
import json
import jsonschema
import threading
import yaml
from datetime import datetime, timezone

class VDA5050Manager:
    def __init__(self, package_directory=None):
        self.lock = threading.Lock()        

        package_share_directory = package_directory
        self.json_topic_files = {
            "connection": "connection.json",
            "factsheet": "factsheet.json",
            "state": "state.json",
            "visualization": "visualization.json",
            "instantactions": "instantActions.json",
            "order": "order.json",
            "manipulatorstate": "manipulatorState.json",
            "amrstate": "amrState.json"
        }

        self.schema_topic_files = {
            "connection": "connection.schema",
            "factsheet": "factsheet.schema",
            "state": "state.schema",
            "visualization": "visualization.schema",
            "instantactions": "instantActions.schema",
            "order": "order.schema",
            "manipulatorstate": "manipulatorState.schema",
            "amrstate": "amrState.schema"
        }

        self.schema_instant_action_files = {
            action: "instantActions.schema" for action in [
                "buildMap",
                "controlMap",
                "controlNodelist",
                "controlManipulator",
                "setMapNode"
            ]
        }

        self.json_topic_list = self.json_topic_files.copy()

        self.instant_action_list = {
            action: action for action in self.schema_instant_action_files
        }

        self.json_data = {}
        self.json_paths = {}
        self.schema_data = {}
        self.instant_action_result = {}

        for key, filename in self.json_topic_files.items():
            json_path = os.path.join(package_share_directory, "json_schemas", filename)
            schema_path = os.path.join(package_share_directory, "json_schemas", self.schema_topic_files[key])

            self.json_paths[key] = json_path
            self.schema_data[key] = self.open_schema(schema_path)
            self.json_data[key] = self.load_json(json_path, key)

        for key in self.instant_action_list:
            base = self.load_json(self.json_paths["instantactions"], "instantactions")
            action = base["actions"][0]

            action["actionId"] = key
            action["actionType"] = key
            action["actionDescription"] = "response"
            action["actionParameters"][0]["key"] = "signal"
            action["actionParameters"][0]["value"] = []

            self.instant_action_result[key] = base

        for key, schema_filename in self.schema_instant_action_files.items():
            schema_path = os.path.join(package_share_directory, "json_schemas", schema_filename)
            self.schema_data[key] = self.open_schema(schema_path)
            
        # print(self.instant_action_result)
        

    def open_schema(self, schema_file_path):
        try:
            with open(schema_file_path, "r") as schema_file:
                return json.load(schema_file)
        except (FileNotFoundError, json.JSONDecodeError):
            return None
        
    def load_json(self, file_path, key):
        try:
            with open(file_path, "r") as json_file:
                data = json.load(json_file)
                if self.validate_json(data, key):
                    return data
                else:
                    print(f"JSON {key} schema error.")
                    return {}
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def save_json(self, key):
        if key in self.json_data:
            file_path = self.json_paths[key]
            with open(file_path, "w") as json_file:
                json.dump(self.json_data[key], json_file, indent=4)

    def update_json_data(self, key, new_data):
        with self.lock:
            if key in self.json_data:
                merged_data = {**self.json_data[key], **new_data}
                if self.validate_json(merged_data, key):
                    self.json_data[key] = merged_data
                else:
                    print(f"fail {key} JSON data update / schema error")

            elif key in self.instant_action_result:
                merged = {**self.instant_action_result[key], **new_data}
                if self.validate_json(merged, key):
                    self.instant_action_result[key] = merged
                else:
                    print(f"fail {key} instant action update / schema error")

            else:
                print(f"Unknown key: {key}")

    def update_json_data_subkey(self, key, subkey, value):
        with self.lock:
            if key in self.json_data:
                original = self.json_data[key].copy()
                original[subkey] = value

                if self.validate_json(original, key):
                    self.json_data[key] = original
                    # self.save_json(key)
                else:
                    print(f"fail {key} JSON subkey update / schema error")

            elif key in self.instant_action_result:
                original = self.instant_action_result[key].copy()
                original[subkey] = value

                if self.validate_json(original, key):
                    self.instant_action_result[key] = original
                else:
                    print(f"fail {key} instant action subkey update / schema error")

            else:
                print(f"Unknown key: {key}")

    def update_json_data_subkeys(self, key, updates: dict):
        with self.lock:
            if key in self.json_data:
                for subkey, value in updates.items():
                    self.json_data[key][subkey] = value

                if self.validate_json(self.json_data[key], key):
                    pass
                else:
                    print(f"fail {key} JSON subkeys update / schema error")

            elif key in self.instant_action_result:
                for subkey, value in updates.items():
                    self.instant_action_result[key][subkey] = value

                if self.validate_json(self.instant_action_result[key], key):
                    pass
                else:
                    print(f"fail {key} instant action subkeys update / schema error")

            else:
                print(f"Unknown key: {key}")

    def update_json_header_time(self, key):
        with self.lock:
            if key in self.json_data:
                self.json_data[key]["headerId"] += 1
                self.json_data[key]["timestamp"] = self.get_current_timestamp()
                return self.json_data[key]

            elif key in self.instant_action_result:
                self.instant_action_result[key]["headerId"] += 1
                self.instant_action_result[key]["timestamp"] = self.get_current_timestamp()
                return self.instant_action_result[key]

            else:
                print(f"'{key}' is not a valid key.")
                return None
    
    def get_current_timestamp(self):
        now = datetime.now(timezone.utc)
        return now.isoformat(timespec='milliseconds').replace("+00:00", "Z")

    def validate_json(self, data, key):
        schema = self.schema_data.get(key)
        if not schema:
            print(f"{key} JSON schema file not found")
            return True

        try:
            jsonschema.validate(data, schema)
            return True
        except jsonschema.exceptions.ValidationError as e:
            print(f"{key} JSON schema error: {e.message}")
            return False 

    def get_json_data(self, key):
        with self.lock:
            if key in self.json_data:
                return self.json_data[key]
            elif key in self.instant_action_result:
                return self.instant_action_result[key]
            else:
                print(f"[WARN] '{key}' not found in json_data or instant_action_result.")
                return {}