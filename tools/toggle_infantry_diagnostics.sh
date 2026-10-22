#!/usr/bin/env bash
set -euo pipefail

project_dir="/home/rm/sp_vision"
config_path="${project_dir}/configs/standard.yaml"
config_key="infantry_diagnostics_enabled"

if [[ ! -f "${config_path}" ]]; then
  notify-send --urgency=critical "日志模式切换失败" "找不到 ${config_path}" || true
  exit 1
fi

# Count the key independently of its value so malformed duplicate keys cannot
# be hidden by the value matcher. Configs edited on Windows may use CRLF.
matching_lines=$(grep -Ec "^${config_key}[[:blank:]]*:" "${config_path}" || true)
if [[ "${matching_lines}" -ne 1 ]]; then
  notify-send --urgency=critical "日志模式切换失败" \
    "${config_key} 必须在正式配置中且只能出现一次" || true
  exit 1
fi

value_pattern="^(${config_key}[[:blank:]]*:[[:blank:]]*)(true|false)([[:blank:]]*(#.*)?\r?)$"
current_value=$(sed -nE "s/${value_pattern}/\2/p" "${config_path}")
if [[ "${current_value}" != "true" && "${current_value}" != "false" ]]; then
  notify-send --urgency=critical "日志模式切换失败" \
    "${config_key} 必须为 true 或 false" || true
  exit 1
fi
if [[ "${current_value}" == "true" ]]; then
  next_value="false"
  mode_name="简洁模式"
  mode_detail="仅输出简洁状态；重启自瞄程序后生效"
else
  next_value="true"
  mode_name="Debug 完整模式"
  mode_detail="记录完整诊断；重启自瞄程序后生效"
fi

# Preserve indentation, inline comments and the original LF/CRLF line ending.
sed -i -E "s/${value_pattern}/\1${next_value}\3/" "${config_path}"

echo "日志模式已切换为：${mode_name}"
echo "${mode_detail}"
notify-send "自瞄日志：${mode_name}" "${mode_detail}" || true
