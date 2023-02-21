#!/usr/bin/python3
# -- coding: utf-8 --
# @Author : Long.Hou
# @Email : Long2.Hou@luxshare-ict.com
# @File : demo2.py
# @Project : TestPlatform
# @Time : 2022/12/6 11:50
# -------------------------------
import os
import sys
import stat
from threading import Thread


class CreateBundle(object):
    """
    create a bundle for mac os, input app_name(which can not have space), version, and start_up_file.py
    """

    def __init__(self, app_name, version, start_up_file, bundle_identifier='luxshare-ict'):
        self._app_path = '../' + app_name + '.app'

        os.system(r'rm -rf "{}"'.format(self._app_path))

        os.makedirs(self._app_path + "/Contents/MacOS")

        self._start_up_file = start_up_file
        self._bundle_name = app_name
        self._version = version
        self._bundle_identifier = bundle_identifier
        self.write_info_plist()
        self.write_pkg_info()
        self.cp_all_folders()
        self.delete_all_py()
        self.create_app_script()
        self.make_script_executable()
        self.mv_resources_folder()

    def write_info_plist(self):
        with open(self._app_path + "/Contents/Info.plist", "w") as f:
            f.write("""<?xml version="1.0" encoding="UTF-8"?>
            <!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
            <plist version="1.0">
            <dict>
                <key>CFBundleDevelopmentRegion</key>
                <string>English</string>
                <key>CFBundleExecutable</key>
                <string>main</string>
                <key>CFBundleGetInfoString</key>
                <string>%s</string>
                <key>CFBundleIconFile</key>
                <string>AppIcon.icns</string>
                <key>CFBundleIdentifier</key>
                <string>%s</string>
                <key>CFBundleInfoDictionaryVersion</key>
                <string>6.0</string>
                <key>CFBundleName</key>
                <string>%s</string>
                <key>CFBundlePackageType</key>
                <string>APPL</string>
                <key>CFBundleShortVersionString</key>
                <string>%s</string>
                <key>CFBundleSignature</key>
                <string>????</string>
                <key>CFBundleVersion</key>
                <string>%s</string>
                <key>NSAppleScriptEnabled</key>
                <string>YES</string>
                <key>NSMainNibFile</key>
                <string>MainMenu</string>
                <key>NSPrincipalClass</key>
                <string>NSApplication</string>
            </dict>
            </plist>
            """ % (self._bundle_identifier + " " + self._version, self._bundle_identifier,
                   self._bundle_name, self._bundle_name + " " + self._version, self._version))

    def write_pkg_info(self):
        with open(self._app_path + "/Contents/PkgInfo", "w") as f:
            f.write("APPL????")

    def make_script_executable(self):
        oldmode = os.stat(self._app_path + "/Contents/MacOS/{}".format(self._bundle_name)).st_mode
        os.chmod(self._app_path + "/Contents/MacOS/{}".format('main'),
                 oldmode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    def cp_all_folders(self):
        os.makedirs("{0}/Contents/Resources/".format(self._app_path))
        cmd = r'cp -a ./. "{}/Contents/Resources/.code"'.format(self._app_path)
        os.system(cmd)

    def mv_resources_folder(self):

        cmd = r'mv {0}/Contents/Resources/.code/AppIcon.icns "{0}/Contents/Resources/"'.format(
            self._app_path)
        os.system(cmd)
        cmd = r'rm -rf {0}/Contents/Resources/.code/.git {0}/Contents/Resources/.code/.gitignore {0}/Contents/Resources/.code/.idea {0}/Contents/Resources/.code/.DS_Store'.format(
            self._app_path)
        os.system(cmd)

    def delete_all_py(self):
        code_path = "{}/Contents/Resources/.code/GUI".format(self._app_path)
        self.delete_py_in_path(code_path)
        # sub folders of code
        # for p in os.listdir(code_path):
        #     # check if folder
        #     if os.path.isdir(p):
        #         cur_path = os.path.join(code_path, p)
        #         # remove all *.py file in this folder
        #         cmd = r'find %s -name "*.py" -exec rm -f {} \;' % cur_path
        #         os.system(cmd)
        #         # for item in os.listdir(cur_path):
        #         #     if item.endswith('.py'):
        #         #         os.remove('%s' % os.path.join(cur_path, item))

    def delete_py_in_path(self, py_path):
        if os.path.isdir(py_path):
            for name in os.listdir(py_path):
                p = os.path.join(py_path, name)
                Thread(target=self.delete_py_in_path, args=(p,)).start()
        elif os.path.isfile(py_path):
            file_name_without_suffix = os.path.splitext(os.path.basename(py_path))[0]
            if str(py_path).endswith('.py'):
                pyc_path = os.path.join(os.path.dirname(py_path), file_name_without_suffix + '.pyc')
                if os.path.exists(pyc_path):
                    os.remove(py_path)

    def create_app_script(self):
        new_python = os.path.join(self._app_path, "Contents", "MacOS", self._bundle_name)
        python_path = os.path.realpath(sys.executable)
        os.symlink(python_path, new_python)
        shell = os.path.join(self._app_path, "Contents", "MacOS", "main")
        with open(shell, 'w') as f:
            f.write("""#!/bin/bash\ncd $(dirname "${BASH_SOURCE[0]}")/../Resources/.code\ntarget=$PWD"/%s"\n../../MacOS/%s $target\n
            """ % (self._start_up_file, self._bundle_name))


if __name__ == '__main__':
    bundle = CreateBundle('H28_MIC', '1.0.01', 'main.py')
    # print(os.path.realpath(sys.executable))