import subprocess, os

def on_upload(source, target, env):
    firmware_path = str(source[0])
    bin_path = os.path.dirname(firmware_path)
    elrs_bin_target = os.path.join(bin_path, 'firmware.elrs')
    cmd = ["curl", "-v", "--max-time", "60",
           "--retry", "2", "--retry-delay", "1",
           "-F", "data=@%s" % (elrs_bin_target,)]
    try:
        print(" ** UPLOADING TO: http://elrs_tx/upload")
        subprocess.check_call(cmd + ["http://elrs_tx/upload"])
        return
    except subprocess.CalledProcessError:
        print("FAILED!")

    print(" ** UPLOADING TO: http://elrs_tx.local/upload")
    try:
        subprocess.check_call(cmd + ["http://elrs_tx.local/upload"])
    except subprocess.CalledProcessError:
        raise Exception("WiFI upload FAILED!")
