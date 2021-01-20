import gzip
import shutil
import os

#
# Code is copied from:
# https://gist.github.com/andrewwalters/d4e3539319e55fc980db1ba67254d7ed
#
def compressFirmware(source, target, env):
    """ Compress ESP8266 firmware using gzip for 'compressed OTA upload' """
    SOURCE_FILE = env.subst("$BUILD_DIR") + os.sep + env.subst("$PROGNAME") + ".bin"
    SOURCE_BAK = SOURCE_FILE + '.bak'
    do_compress = True
    if os.path.exists(SOURCE_FILE) and os.path.exists(SOURCE_BAK):
        src_mtime = os.stat(SOURCE_FILE).st_mtime
        bak_mtime = os.stat(SOURCE_BAK).st_mtime
        """ Recompress if .bin file is newer than .bak file """
        do_compress = (src_mtime > bak_mtime)

    if do_compress:
        print("Compressing firmware for upload...")
        shutil.move(SOURCE_FILE, SOURCE_BAK)
        with open(SOURCE_BAK, 'rb') as f_in:
            with gzip.open(SOURCE_FILE, 'wb') as f_out:
                shutil.copyfileobj(f_in, f_out)
        """ Set modification time on compressed file so incremental build works """
        shutil.copystat(SOURCE_BAK, SOURCE_FILE)

    if os.path.exists(SOURCE_FILE+'.bak'):
        ORG_FIRMWARE_SIZE = os.stat(SOURCE_FILE + '.bak').st_size
        GZ_FIRMWARE_SIZE = os.stat(SOURCE_FILE).st_size

        print("Compression reduced firmware size by {:.0f}% (was {} bytes, now {} bytes)".format(
              (GZ_FIRMWARE_SIZE / ORG_FIRMWARE_SIZE) * 100, ORG_FIRMWARE_SIZE, GZ_FIRMWARE_SIZE))
