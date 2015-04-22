#!/usr/bin/env python

# Usage: progname <directory>
#
# This program scans <directory> and generates C code on its stdout
# that contains an array of psf_entry structures, one for each file
# in <directory>.  This code may be used to instantiate a file in procfs
# whose contents is identical to each file in <directory>
#
# Author: Dale Farnsworth <dale@farnsworth.org>
#
# Copyright 2009 (c) MontaVista Software, Inc. This file is licensed
# under the terms of the GNU General Public License version 2.
# This program is licensed "as is" without any warranty of any kind,
# whether express or implied.

import sys
import os
import glob
import subprocess

class Entry:
	def __init__(self, path, parent):
		self.path = path
		self.parent = parent
		self.isdir = os.path.isdir(path)

	@classmethod
	def all_entries(self, path=""):
		def recurse(parent_path, parent):
			for path in glob.glob(os.path.join(parent_path, "*")):
				entry = Entry(path, parent)
				entry.index = len(entries)
				entries.append(entry)
				if entry.isdir:
					recurse(path, entry)

		entries = []
		recurse(path, None)
		return entries

def output_file_data(entries):
	for entry in entries:
		if entry.isdir:
			continue

		sys.stdout.write("/* %s */\n" % entry.path)
		sys.stdout.write("static char data_%s[] = \n" % entry.index)
		sys.stdout.flush()
		bin2c = os.path.join(os.environ["objtree"], "scripts/bin2c")
		f = open(entry.path)
		subprocess.call([bin2c], stdin=f)
		f.close()
		sys.stdout.write("\t;\n\n")

def output_psf_entries(entries):
	sys.stdout.write("static struct psf_entry psf_table[] = {\n")

	for entry in entries:
		if entry.parent:
			parent_addr = "&psf_table[%d]" % entry.parent.index
		else:
			parent_addr = "NULL"

		if entry.isdir:
			data = "NULL"
			size = "-1"
		else:
			data = "data_%d" % entry.index
			size = "sizeof(%s) - 1" % data

		sys.stdout.write(' /*%d*/\t{ "%s", %s, %s, %s },\n' %
					(entry.index,
					 os.path.basename(entry.path),
					 parent_addr, data, size))

	sys.stdout.write("};\n")

def main():
	progname = os.path.basename(sys.argv[0])

	if len(sys.argv) < 2:
		sys.stderr.write("Usage: %s <directory>\n" % progname)
		sys.exit(1)

	dir = sys.argv[1]
	if not os.path.isdir(dir):
		sys.stderr.write("%s: %s: not a directory\n" % (progname, dir))
		sys.exit(1)

	os.chdir(dir)

	entries = Entry.all_entries()

	output_file_data(entries)
	output_psf_entries(entries)

main()
