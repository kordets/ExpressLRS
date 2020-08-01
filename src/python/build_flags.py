Import("env")
import os
import fhss_random
import hashlib

def parse_flags(path):
    domains_found = 0
    domains_found_ism = 0
    build_flags = env['BUILD_FLAGS']
    ISM2400 = False
    for flag in build_flags:
        if "DOMAIN_24GHZ" in flag:
            ISM2400 = True
            break
    try:
        with open(path, "r") as _f:
            for line in _f:
                define = line.strip()
                if define.startswith("-D"):
                    if "MY_UID" in define and len(define.split(",")) != 6:
                        raise Exception("UID must be 6 bytes long")
                    elif "MY_PHRASE" in define:
                        define = define.split("=")[1]
                        define = define.replace('"', '').replace("'", "")
                        key = define.replace("-D", "")
                        if len(define) < 8:
                            raise Exception("MY_PHRASE must be at least 8 characters long")
                        md5 = hashlib.md5(key.encode()).digest()
                        define = "-DMY_UID=" + ",".join(["0x%02X"%r for r in md5[:6]])
                    elif "Regulatory_Domain" in define:
                        if "_ISM_2400" in define:
                            domains_found_ism += 1
                            if not ISM2400:
                                continue
                            #print("\n\033[93m[NOTE] ISM 2400 band selected, Using SX128x radio!\n")
                            build_flags.append("-DRADIO_SX128x=1")
                            if "_800kHz" in define:
                                build_flags.append("-DRADIO_SX128x_BW800=1")
                        elif ISM2400:
                            domains_found += 1
                            continue
                        if domains_found > 1 or domains_found_ism > 1:
                            raise Exception("[ERROR] Only one 'Regulatory_Domain' is allowed")
                    build_flags.append(define)
    except IOError:
        return False
    return True

if not parse_flags("user_defines_private.txt"):
    if not parse_flags("user_defines.txt"):
        err = "\n\033[91m[ERROR] File '%s' does not exist\n" % user_defines.txt
        raise Exception(err)

print("\n[INFO] build flags: %s\n" % env['BUILD_FLAGS'])

fhss_random.check_env_and_parse(env['BUILD_FLAGS'])
