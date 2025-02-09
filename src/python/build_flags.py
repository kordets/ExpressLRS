Import("env")
import os
import fhss_random
import hashlib
try:
    import git
except ImportError:
    env.Execute('"$PYTHONEXE" -m pip install GitPython')
    try:
        import git
    except ImportError:
        git = None


def parse_flags(path):
    domains_found = []
    domains_found_ism = []
    build_flags = env['BUILD_FLAGS']
    ISM2400 = DUAL_MODE = False
    for flag in build_flags:
        if "DOMAIN_24GHZ" in flag:
            ISM2400 = True
        elif "DOMAIN_BOTH" in flag:
            DUAL_MODE = True
    try:
        with open(path, "r") as _f:
            for line in _f:
                define = line.strip()
                if define.startswith("-D"):
                    is_uid = "MY_PHRASE" in define or "MY_UID" in define

                    define_key = define.split("=")[0]
                    build_flags_copy = list(build_flags)
                    for flag in build_flags_copy:
                        if define_key in flag or (is_uid and ("MY_PHRASE" in flag or "MY_UID" in flag)):
                            ###print("remove %s (%s, %s)" % (flag, define_key, define))
                            # remove value and it will be replaced
                            build_flags.remove(flag)
                            break

                    if "MY_PHRASE" in define:
                        define = define.split("=")[1]
                        define = define.replace('"', '').replace("'", "")
                        key = define.replace("-D", "")
                        if len(define) < 8:
                            raise Exception("MY_PHRASE must be at least 8 characters long")
                        md5 = hashlib.md5(key.encode()).hexdigest()
                        print("Hash value: %s" % md5)
                        #my_uid = ["0x%02X"%ord(r) for r in md5[:6]]
                        my_uid = ["0x%02X"%int(md5[i:(i+2)],16) for i in range(0, 12, 2)]
                        define = "-DMY_UID=" + ",".join(my_uid)
                        print("Calculated UID[6] = {%s}" % ",".join(my_uid))
                    elif "MY_UID" in define and len(define.split(",")) != 6:
                        raise Exception("UID must be 6 bytes long")
                    elif "Regulatory_Domain" in define:
                        if "_ISM_2400" in define:
                            if not DUAL_MODE and not ISM2400:
                                continue
                            domains_found_ism.append(define)
                        else:
                            if not DUAL_MODE and ISM2400:
                                continue
                            domains_found.append(define)
                    build_flags.append(define)

        if len(domains_found) > 1 or len(domains_found_ism) > 1:
            raise Exception("[ERROR] Only one 'Regulatory_Domain' is allowed")
        if domains_found:
            build_flags.append("-DRADIO_SX127x=1")
        if domains_found_ism:
            build_flags.append("-DRADIO_SX128x=1")
            if "_800kHz" in domains_found_ism[0]:
                build_flags.append("-DRADIO_SX128x_BW800=1")
    except IOError:
        return False
    return True

if not parse_flags("user_defines.txt"):
    err = "\n\033[91m[ERROR] File 'user_defines.txt' does not exist\n"
    raise Exception(err)
# try to parse user private params
parse_flags("user_defines_private.txt")

sha = None
if git:
    try:
        git_repo = git.Repo(
            os.path.abspath(os.path.join(os.getcwd(), os.pardir)),
            search_parent_directories=False)
        git_root = git_repo.git.rev_parse("--show-toplevel")
        ExLRS_Repo = git.Repo(git_root)
        hexsha = ExLRS_Repo.head.object.hexsha
        sha = ",".join(["0x%s" % x for x in hexsha[:6]])
    except git.InvalidGitRepositoryError:
        pass
if not sha:
    if os.path.exists("VERSION"):
        with open("VERSION") as _f:
            data = _f.readline()
            _f.close()
        hexsha = data.split()[1].strip()
        sha = ",".join(["0x%s" % x for x in hexsha[:6]])
    else:
        sha = "0,0,0,0,0,0"
print("Current SHA: %s" % sha)
env['BUILD_FLAGS'].append("-DLATEST_COMMIT="+sha)

print("\n[INFO] build flags: %s\n" % env['BUILD_FLAGS'])

fhss_random.check_env_and_parse(env['BUILD_FLAGS'])

# Set upload_protovol = 'custom' for STM32 MCUs
#  otherwise firmware.bin is not generated
stm = env.get('PIOPLATFORM', '') in ['ststm32']
if stm:
    env['UPLOAD_PROTOCOL'] = 'custom'
