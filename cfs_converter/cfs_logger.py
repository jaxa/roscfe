"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import datetime


class CfsLogger:
    """Output logs."""

    logfp = None

    def __init__(self, log_dir_path):
        """Constructor.

        Args:
            log_dir_path (str): Path to the directory of logs

        """
        now = datetime.datetime.now()
        logfile_path = log_dir_path + "/" \
            + now.strftime("%Y%m%d_%H%M%S") + ".log"
        self.logfp = open(logfile_path, "a")

    def _dumpLog(self, original_line, line, cfe_src_file):
        """Dump log.

        Args:
            original_line (str): Original line of source
            line (str): Converted line
            cfe_src_file (str): Source file name of cFE app

        """
        if original_line == line:
            return
        message = cfe_src_file \
            + "Substitute :: [{}] to [{}]\n".format(original_line, line)
        self.logfp.write(message)
        print(message)

    def _close(self):
        """Close Cfs_Logger."""
        self.logfp.close()
