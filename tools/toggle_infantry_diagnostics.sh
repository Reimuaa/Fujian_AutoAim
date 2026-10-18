#!/usr/bin/env bash
set -euo pipefail

project_dir="/home/rm/sp_vision"
config_path="${project_dir}/configs/standard.yaml"
config_key="infantry_diagnostics_enabled"

if [[ ! -f "${config_path}" ]]; then
  notify-send --urgency=critical "日志模式切换失败" "找不到 ${config_path}" || true
  exit 1
fi

matching_lines=$(grep -Ec "^${config_key}: (true|false)$" "${config_path}" || true)
if [[ "${matching_lines}" -ne 1 ]]; then
  notify-send --urgency=critical "日志模式切换失败" \
    "${config_key} 必须在正式配置中且只能出现一次" || true
  exit 1
fi

current_value=$(awk -v key="${config_key}" '$1 == key ":" {print $2}' "${config_path}")
if [[ "${current_value}" == "true" ]]; then
  next_value="false"
  mode_name="简洁模式"
  mode_detail="仅输出简洁状态；重启自瞄程序后生效"
else
  next_value="true"
  mode_name="Debug 完整模式"
  mode_detail="记录完整诊断；重启自瞄程序后生效"
fi

sed -i "s/^${config_key}: ${current_value}$/${config_key}: ${next_value}/" "${config_path}"

echo "日志模式已切换为：${mode_name}"
echo "${mode_detail}"
notify-send "自瞄日志：${mode_name}" "${mode_detail}" || true
