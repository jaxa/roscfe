"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import datetime


class RelayLogger:
    """Relay logger."""

    logfp = None

    def __init__(self, log_dir_path):
        """Constructor.

        Args:
            log_dir_path (str): Path to the log directory

        """

        now = datetime.datetime.now()
        logfile_path = log_dir_path + "/relay_" + \
            now.strftime("%Y%m%d_%H%M%S") + ".log"
        self.logfp = open(logfile_path, "a")

    def _dumpLog(self, msg):
        """Dump log message.

        Args:
            msg (str): Dump message

        """

        self.logfp.write(msg + "\n")
        print(msg)

    def _close(self):
        """Close relay_logger."""

        self.logfp.close()
