# This file is to setup the visualization tools for pycram notebooks.
# Will be run automatically when opening a notebook on BinderHub.

import os
from IPython.display import display, HTML
from sidecar import Sidecar
import subprocess
import threading
from time import sleep

# Display remote desktop on sidecar tab
def display_desktop():
    try:
        JUPYTERHUB_USER = os.environ['JUPYTERHUB_USER']
    except KeyError:
        JUPYTERHUB_USER = None
    url_prefix = f"/user/{JUPYTERHUB_USER}" if JUPYTERHUB_USER is not None else ''
    remote_desktop_url = f"{url_prefix}/desktop"
    sc = Sidecar(title='Desktop', anchor="split-right")
    with sc:
        # The inserted custom HTML and CSS snippets are to make the tab resizable
        display(HTML(f"""
            <style>
            body.p-mod-override-cursor div.iframe-widget {{
                position: relative;
                pointer-events: none;

            }}

            body.p-mod-override-cursor div.iframe-widget:before {{
                content: '';
                position: absolute;
                top: 0;
                left: 0;
                right: 0;
                bottom: 0;
                background: transparent;
            }}
            </style>
            <div class="iframe-widget" style="width: calc(100% + 10px);height:100%;">
                <iframe src="{remote_desktop_url}" width="100%" height="100%"></iframe>
            </div>
        """))

# Run bash command in the background
def run_background_command(cmd):
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setpgrp
    )
    print(f"Started process with PID {process.pid}")
    process.wait()

# Run rivz2 in the background
def launch_rviz(config='pr2.rviz'):
    is_rviz_running = False
    try:
        output = subprocess.check_output(["pgrep", "-x", "rviz"])
        is_rviz_running =  bool(output.strip())
    except subprocess.CalledProcessError:
        is_rviz_running = False

    if is_rviz_running == False:
        thread = threading.Thread(target=run_background_command, kwargs={
            "cmd":["rviz", "--disable-anti-aliasing", "--fullscreen", "-d", config]
            # "cmd":["rviz", "--fullscreen", "-d", config]
        }, daemon=True)
        thread.start()
        
def launch_sim(launchfile="pr2_mujoco.launch"):
    LAUNCH_FILE_DIR = os.path.abspath(os.path.join(os.getcwd(), "../launch"))
    LAUNCH_CMD = ["roslaunch",
        os.path.join(LAUNCH_FILE_DIR, launchfile),
        "mujoco_suffix:=_headless"]
    try:
        subprocess.run(["pkill", "-f", launchfile], check=False)
    except Exception as e:
        pass
    thread = threading.Thread(target=run_background_command, kwargs={
        "cmd": LAUNCH_CMD
    }, daemon=True)
    thread.start()

launch_sim()
launch_rviz()
