# -*- coding: utf-8 -*-
from __future__ import print_function

import logging
import subprocess
import sys

from . import arguments
from . import run_components


def main(a):
    args = arguments.parse_args(a)
    logging.basicConfig(level=getattr(logging, args.log_level.upper()),
                        format="%(levelname)-8s %(message)s",
                        stream=sys.stdout)
    logging.debug("processed args: %s" % args)

    log = ""

    for component in args.components:
        if component == "translate":
            log += run_components.run_translate(args)
        elif component == "preprocess":
            log += run_components.run_preprocess(args)
        elif component == "search":
            log += run_components.run_search(args)
        else:
            assert False
    return log



if __name__ == "__main__":
    main()
