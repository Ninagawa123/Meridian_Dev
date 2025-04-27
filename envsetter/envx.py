#!/usr/bin/env python3
"""
envx.py  ──  クロスプラットフォーム環境変数マネージャ
  list            すべてのレイヤーをマージして表示
  set   NAME VAL  変数を設定（--layer で範囲指定）
  unset NAME      変数を削除（--layer で範囲指定）

  --json     list を JSON 出力
  --sources  list の行末に取得元を表示
  --layer    shell / persistent / both   ← set / unset 用

◾ 使い方
# すべての環境変数を取得
envx list --sources | head

# 変数 FOO をシェル RC と永続レイヤーに同時設定
envx set FOO bar

# Windows だけ永続レイヤーを変更したい場合
envx set APIKEY 123 --layer persistent

# Ubuntu で .bashrc だけを変更したい場合
envx unset FOO --layer shell

◾ インストール手順
macOS / Ubuntu / WSL(※インストールについては動作未確認)
bash:
curl -fsSL https://raw.githubusercontent.com/your/repo/main/envx.py | sudo tee /usr/local/bin/envx >/dev/null && sudo chmod +x /usr/local/bin/envx

Windows (PowerShell)(※インストールについては動作未確認)
iwr -UseBasicParsing https://raw.githubusercontent.com/your/repo/main/envx.py -OutFile "$env:USERPROFILE\envx.py"; if ($env:PATH -notlike "*$env:USERPROFILE*") { setx PATH "$env:PATH;$env:USERPROFILE" }

GPT:https://chatgpt.com/share/680ebde6-6f88-800f-b1eb-1a21e63a6f15
"""
import os
import sys
import json
import argparse
import platform
import subprocess
import re
from pathlib import Path

OS = platform.system()
IS_WIN = OS == "Windows"
IS_MAC = OS == "Darwin"
IS_LINUX = OS == "Linux"

# ----------------------------------------------------------------------
# 取得レイヤー
# ----------------------------------------------------------------------


def get_current_env():
    return dict(os.environ)


def get_login_shell_env():
    if IS_WIN:           # Windows には login-shell という概念がない
        return {}
    shell = os.environ.get("SHELL") or "/bin/bash"
    try:
        out = subprocess.check_output(
            [shell, "-lc", "printenv -0"], stderr=subprocess.DEVNULL)
        return dict(x.decode().split("=", 1) for x in out.split(b"\0") if b"=" in x)
    except subprocess.CalledProcessError:
        return {}


def get_persistent_env():
    """
    - macOS  : launchctl print gui/$UID の environment ブロック
    - Linux  : ~/.config/systemd/user-environment.d/*.conf を読む（無ければ空）
    - Windows: HKCU\Environment レジストリ
    """
    if IS_MAC:
        uid = os.getuid()
        try:
            out = subprocess.check_output(
                ["launchctl", "print", f"gui/{uid}"], stderr=subprocess.DEVNULL, text=True
            )
        except subprocess.CalledProcessError:
            return {}
        env, inblk = {}, False
        for ln in out.splitlines():
            if ln.strip().startswith("environment"):
                inblk = True
                continue
            if inblk:
                if not ln.startswith("\t\t"):
                    break
                m = re.match(r"\t\t([^=]+) = (.*)", ln)
                if m:
                    env[m.group(1)] = m.group(2).strip('"')
        return env

    elif IS_LINUX:
        # systemd-user の drop-in (22.04 以降) に対応
        cfg_dir = Path.home()/".config/systemd/user-environment.d"
        env = {}
        for conf in cfg_dir.glob("*.conf"):
            for ln in conf.read_text().splitlines():
                if not ln or ln.lstrip().startswith("#"):
                    continue
                k, _, v = ln.partition("=")
                env[k.strip()] = v.strip().strip('"')
        return env

    elif IS_WIN:
        import winreg
        env = {}
        try:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Environment") as k:
                i = 0
                while True:
                    try:
                        name, val, _ = winreg.EnumValue(k, i)
                        env[name] = val
                        i += 1
                    except OSError:
                        break
        except OSError:
            pass
        return env
    return {}

# ----------------------------------------------------------------------
# shell RC ファイル自動選択
# ----------------------------------------------------------------------


def choose_rc_file():
    if IS_WIN:
        return None
    sh = Path(os.environ.get("SHELL", "/bin/bash")).name
    return {
        "zsh":  Path.home()/".zshrc",
        "bash": Path.home()/".bash_profile",
    }.get(sh, Path.home()/".profile")

# ----------------------------------------------------------------------
# shell レイヤー編集（macOS / Linux）
# ----------------------------------------------------------------------


def modify_rc(name, value):
    rc = choose_rc_file()
    if not rc:
        return False
    text = rc.read_text() if rc.exists() else ""
    pat = re.compile(rf'^\s*export\s+{re.escape(name)}=.*$', re.MULTILINE)
    if value is None:
        new, n = pat.subn("", text)
        if n == 0:
            return False
    else:
        line = f'export {name}="{value}"'
        new = pat.sub(line, text) if pat.search(
            text) else text.rstrip()+"\n"+line+"\n"
    rc.write_text(new)
    return True

# ----------------------------------------------------------------------
# persistent レイヤー編集
# ----------------------------------------------------------------------


def set_persistent(name, value):
    if IS_MAC:
        cmd = ["launchctl", "setenv", name, value]
    elif IS_LINUX:
        cfg = Path.home()/".config/systemd/user-environment.d/99-envx.conf"
        lines = []
        if cfg.exists():
            lines = [ln for ln in cfg.read_text().splitlines()
                     if not ln.startswith(f"{name}=")]
        if value is not None:
            lines.append(f'{name}="{value}"')
        cfg.parent.mkdir(parents=True, exist_ok=True)
        cfg.write_text("\n".join(lines)+"\n")
        # 即時反映
        cmd = ["systemctl", "--user", "import-environment",
               name] if value else ["systemctl", "--user", "unset-environment", name]
    elif IS_WIN:
        import winreg
        import ctypes
        if value is None:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Environment", 0, winreg.KEY_SET_VALUE) as k:
                try:
                    winreg.DeleteValue(k, name)
                except FileNotFoundError:
                    pass
        else:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r"Environment", 0, winreg.KEY_SET_VALUE) as k:
                winreg.SetValueEx(k, name, 0, winreg.REG_EXPAND_SZ, value)
        # ブロードキャストして即座に反映（再ログイン不要）
        ctypes.windll.user32.SendMessageTimeoutW(
            0xFFFF, 0x1A, 0, u"Environment", 0, 1000, None)
        return True
    else:
        return False
    try:
        subprocess.check_call(cmd, stderr=subprocess.DEVNULL)
        return True
    except Exception:
        return False

# ----------------------------------------------------------------------
# マージ
# ----------------------------------------------------------------------


def collect_all():
    merged = {}
    for layer, env in [
        ("current", get_current_env()),
        ("login-shell", get_login_shell_env()),
        ("persistent", get_persistent_env()),
    ]:
        for k, v in env.items():
            merged.setdefault(k, {"value": v, "source": layer})
    return merged

# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------


def main():
    cli = argparse.ArgumentParser()
    sub = cli.add_subparsers(dest="cmd", required=True)

    # list
    p_list = sub.add_parser("list")
    p_list.add_argument("--json", action="store_true")
    p_list.add_argument("--sources", action="store_true")

    # set
    p_set = sub.add_parser("set")
    p_set.add_argument("name")
    p_set.add_argument("value")
    p_set.add_argument(
        "--layer", choices=["shell", "persistent", "both"], default="both")

    # unset
    p_uns = sub.add_parser("unset")
    p_uns.add_argument("name")
    p_uns.add_argument(
        "--layer", choices=["shell", "persistent", "both"], default="both")

    args = cli.parse_args()

    if args.cmd == "list":
        env = collect_all()
        if args.json:
            json.dump({k: v["value"]
                      for k, v in env.items()}, sys.stdout, indent=2)
            print()
        else:
            for k in sorted(env):
                rec = env[k]
                line = f"{k}={rec['value']}"
                if args.sources:
                    line += f"    # {rec['source']}"
                print(line)
        return

    target = {"shell", "persistent"} if args.layer == "both" else {args.layer}
    success = []

    if args.cmd == "set":
        if "shell" in target and modify_rc(args.name, args.value):
            success.append("shell")
        if "persistent" in target and set_persistent(args.name, args.value):
            success.append("persistent")
        os.environ[args.name] = args.value                         # 現プロセスにも反映

    elif args.cmd == "unset":
        if "shell" in target and modify_rc(args.name, None):
            success.append("shell")
        if "persistent" in target and set_persistent(args.name, None):
            success.append("persistent")
        os.environ.pop(args.name, None)

    print("OK:", ",".join(success) if success else "無変更")


if __name__ == "__main__":
    main()
