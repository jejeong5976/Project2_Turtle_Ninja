#!/bin/bash
# install.sh — 한 번만: ~/.local/bin/nt → tools/robot_cli/robot_cli.sh 심볼릭 링크 (sudo 없이, 젯슨 · PC 같음)
#   그 뒤 어느 폴더에서든 nt (화면) · nt status · nt h · nt g B1 · nt check
set -euo pipefail
SELF="$(readlink -f "${BASH_SOURCE[0]}")"
TARGET="$(dirname "$SELF")/robot_cli.sh"
BIN="$HOME/.local/bin"
LINK="$BIN/nt"
[ -x "$TARGET" ] || { echo "실행 파일 없음: $TARGET"; exit 1; }

existing="$(command -v nt || true)"
if [ -n "$existing" ] && [ "$(readlink -f "$existing")" != "$TARGET" ]; then
  echo "이미 nt 라는 명령이 있음: $existing"
  [ "$existing" = "$LINK" ] || echo "  ($LINK 를 만들어도 PATH 순서에 따라 그쪽이 먼저 불릴 수 있음)"
  read -r -p "덮어써서 이 robot_cli 를 nt 로 할까요? [y/N] " a
  [ "$a" = y ] || [ "$a" = Y ] || { echo "그만둠"; exit 1; }
elif [ -e "$LINK" ] || [ -L "$LINK" ]; then
  if [ "$(readlink -f "$LINK")" = "$TARGET" ]; then echo "이미 설치됨: $LINK → $TARGET"; exit 0; fi
  read -r -p "$LINK 가 이미 있음 ($(readlink -f "$LINK")). 덮어쓸까요? [y/N] " a
  [ "$a" = y ] || [ "$a" = Y ] || { echo "그만둠"; exit 1; }
fi
mkdir -p "$BIN"
ln -sfn "$TARGET" "$LINK"
echo "설치: $LINK → $TARGET"
case ":$PATH:" in
  *":$BIN:"*) echo "이제 nt 로 켬 (nt help = 명령 목록)";;
  *) echo "주의: $BIN 이 PATH 에 없음 → ~/.bashrc 에 다음 줄을 넣고 새 터미널:"
     echo "  export PATH=\"\$HOME/.local/bin:\$PATH\"";;
esac
