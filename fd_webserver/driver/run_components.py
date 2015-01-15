# -*- coding: utf-8 -*-

import logging
import os
import os.path
import subprocess
import sys

from . import portfolio_runner
from .plan_manager import PlanManager


DRIVER_DIR = os.path.abspath('/home/marc/workspace/CMP2020M/install/lib/catkinized_downward') #os.path.abspath(os.path.dirname(__file__))
SRC_DIR = DRIVER_DIR
TRANSLATE = os.path.join(SRC_DIR, "translate", "translate.py")
PREPROCESS = os.path.join(SRC_DIR, "preprocess", "preprocess")
SEARCH_DIR = os.path.join(SRC_DIR, "search")


def call_cmd(cmd, args, debug, cwd, stdin=None):
    if not os.path.exists(cmd):
        target = " debug" if debug else ""
        raise IOError(
            "Could not find %s. Please run \"./build_all%s\"." %
            (cmd, target))
    sys.stdout.flush()
    if stdin:
        with open(stdin) as stdin_file:
            return subprocess.check_output([cmd] + args, cwd=cwd, stdin=stdin_file, stderr=subprocess.STDOUT)
    else:
        return subprocess.check_output([cmd] + args, cwd=cwd, stderr=subprocess.STDOUT)


def run_translate(args):
    logging.info("Running translator.")
    logging.info("translator inputs: %s" % args.translate_inputs)
    logging.info("translator arguments: %s" % args.translate_options)
    return call_cmd(TRANSLATE, args.translate_inputs + args.translate_options,
             debug=args.debug, cwd=args.cwd)


def run_preprocess(args):
    logging.info("Running preprocessor.")
    logging.info("cwd %s" % args.cwd)
    args.preprocess_input = os.path.join(args.cwd,"output.sas")
    logging.info("preprocessor input: %s" % args.preprocess_input)
    logging.info("preprocessor arguments: %s" % args.preprocess_options)
    return call_cmd(PREPROCESS, args.preprocess_options, debug=args.debug, cwd=args.cwd,
             stdin=args.preprocess_input)


def run_search(args):
    logging.info("Running search.")
    args.search_input = os.path.join(args.cwd,"output")
    logging.info("search input: %s" % args.search_input)

    plan_manager = PlanManager(args.plan_file)
    plan_manager.delete_existing_plans()

    if args.debug:
        executable = os.path.join(SEARCH_DIR, "downward-debug")
    else:
        executable = os.path.join(SEARCH_DIR, "downward-release")
    logging.info("search executable: %s" % executable)

    if not args.search_options:
        raise ValueError(
            "search needs search options")
    if "--help" not in args.search_options:
        args.search_options.extend(["--internal-plan-file", args.plan_file])
    logging.info("search arguments: %s" % args.search_options)
    return call_cmd(executable, args.search_options, debug=args.debug, cwd=args.cwd,
             stdin=args.search_input)
