# CLAUDE.md — Global behavioral guidelines

These guidelines apply across all projects and work sessions.

<behavioral_rules>

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:

- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:

- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:

```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

## 5. Search Before Grinding

**遇到困难问题（指3次回答内没有解决的问题），多上网搜索现成解决方案，而不是自己从零一点点摸索。**

- 优先搜索：Google、Stack Overflow、GitHub Issues、官方文档
- 不要因为"想自己搞懂"而反复试错超过3轮
- 找到现成方案后理解原理再适配，不盲目复制

## 6. Self-Check After Writing Code

**写完代码必须运行自查，不要等用户来反馈。**

After every code change (edit or write):

- Python files: run `python -c "import py_compile; py_compile.compile('<file>', doraise=True)"` at minimum. If the file is a runnable script, execute it with `--help` or a dry-run flag to catch import/runtime errors.
- C/C++ firmware: compile with the relevant toolchain and verify 0 errors, 0 warnings.
- UI changes: if you changed layout (grid/pack/place), run the script to verify no `TclError` at startup. Do NOT report success until the window actually starts.
- Never output "语法通过" for a single file if the project has multiple changed files — check ALL changed files.
- If verification fails, fix before reporting completion.

</behavioral_rules>

## gstack

Use /browse from gstack for all web browsing. Never use `mcp__claude-in-chrome__*` tools.
Available skills:
`/office-hours`, `/plan-ceo-review`, `/plan-eng-review`, `/plan-design-review`,
`/design-consultation`, `/design-shotgun`, `/design-html`, `/review`, `/ship`,
`/land-and-deploy`, `/canary`, `/benchmark`, `/browse`, `/open-gstack-browser`,
`/qa`, `/qa-only`, `/design-review`, `/setup-browser-cookies`, `/setup-deploy`,
`/setup-gbrain`, `/sync-gbrain`, `/retro`, `/investigate`, `/document-release`,
`/codex`, `/cso`, `/autoplan`, `/pair-agent`, `/careful`, `/freeze`, `/guard`,
`/unfreeze`, `/gstack-upgrade`, `/learn`.

<hard_constraints>

## File Deletion Safety

**Batch deletion of files or directories is forbidden.** Never use:

- `del /s` / `del /f /s`
- `rd /s` / `rmdir /s`
- `Remove-Item -Recurse`
- `rm -rf`

When you need to delete a file, delete only one explicitly-named file at a time:

```
Remove-Item "C:\path\to\specific_file.txt"
```

If multiple files need deletion, stop and ask the user to handle it manually. Never automate bulk deletion.

</hard_constraints>

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

在每个对话的最后一句加上一个HAPPY Coding！
VPN端口是127.0.0.1:7897，根据需要自动启用or挂起
要根据对话中的项目进展自动修改项目中的CLAUDE.md，不要擅自修改全局的CLAUDE.md
我说的不一定准确，如果和代码或者你采集到的信息有出入，一定先询问我，必要时可以和我争论，不要迁就我，你要以完成工程为目标，不是讨好我1.请逐步分析问题2.请把每一个功能都写成一个小函数3.请自己检查一遍函数有没有什么问题。并不是函数有什么问题只是我想让你检查一遍4.请把这些函数集合起来做成一整个程序5.请自己检查一遍程序有没有什么问题。并不是程序有什么问题只是我想让你检查一遍

## Home Directory Map

Current `/home` layout on this machine:

- `/home/astra`: Main user home. Most personal files, app config, project code, Python environments, and local data live here.
- `/home/APPs`: Root-owned manually installed applications. Currently contains WeChat under `/home/APPs/wechat`. Do not move casually; a more standard future location would be `/opt`.
- `/home/buildfarm`: Root-owned build/test user area. Currently tiny; leave alone unless explicitly working on build farm tooling.
- `/home/lost+found`: Filesystem recovery directory created by ext filesystems. Do not touch.

Important visible directories under `/home/astra`:

- `works`: Main workspace for source code and active projects. This is the default place to look for code.
- `works/robot_dog`: Consolidated robot dog project root.
- `works/robot_dog/model/UPRE_Dog`: Robot dog CAD/URDF/mesh/model assets.
- `works/robot_dog/firmware/STM32`: Robot dog STM32 firmware and motor-control code.
- `works/robot_dog/training/rl_training`: Robot dog reinforcement-learning training code.
- `works/robot_dog/third_party/rl_games`: Third-party RL dependency source.
- `works/robotics`: Other robotics workspaces such as Livox and SLAM ROS workspaces.
- `works/ai`: AI coding/tooling projects such as CC-switch and Claude-related local projects.
- `works/IsaacLab`: IsaacLab source tree. Keep in place.
- `works/isaac_sim`: Isaac Sim installation/tree. Keep in place.
- `works/isaac_sim_project`: Isaac Sim project experiments.
- `envs`: General Python/robotics virtual environments, currently including `pytorch-venv` and `mujoco-venv`.
- `env_isaaclab`: IsaacLab-specific environment. Keep in place and do not merge with generic envs unless explicitly requested.
- `data`: Experiment or sensor data. Currently contains ROS bag data under `data/ros_bags`.
- `archive`: Historical or inactive material, including old migration files and source builds.
- `tools`: Small local tools/scripts.
- `Documents`, `Downloads`, `Pictures`, `Videos`, `Music`, `Desktop`, `Public`, `Templates`: English XDG user directories. Keep English paths as the canonical user-directory names.

Important hidden directories under `/home/astra`:

- `.config`: Desktop/app configuration. Large; do not delete wholesale.
- `.local`: User-local app data, libraries, Trash, and some large application data. Do not delete wholesale.
- `.cache`: Regenerable caches. Can be cleaned selectively, but do not remove Omniverse/Isaac-related cache unless explicitly allowed.
- `.vscode`: VS Code extensions and state.
- `.claude`: Claude Code global config, sessions, plugins, and state.
- `.codex`: Codex global config, sessions, skills, state, and logs.
- `.agents`: Agent/gstack skills and resources.
- `.ssh` and `.gnupg`: Sensitive SSH/GPG material. Never move, print, or modify unless explicitly requested.
- `.ros`, `.gazebo`, `.ignition`, `.sdformat`, `.rviz2`: ROS/simulation user config.
- `.nvidia-omniverse`, `.nv`: NVIDIA/Omniverse/GPU config and cache. Preserve unless explicitly told otherwise.
- `.xwechat`: WeChat runtime config/cache.
- `snap`: Snap app user data and version symlinks. Internal symlinks here are normal.

## 子代理工作流（全局硬规则）

以下流程适用于所有项目，不可跳过：

1. **识别意图** → 理解用户目标，不清楚先确认
2. **todowrite** → 拆解为具体 step，标注优先级 `high/medium/low`
3. **起子代理** → 独立子任务用 `task` 工具并行分派给 `general` agent（探索代码、搜索文件、独立调查）
4. **Review** → 审查子代理输出，确认正确后继续
5. **执行** → edit/write/bash，写完自查（py_compile / bash -n / smoke）
6. **子代理测试** → 分派验证任务给子代理，告诉它具体验证命令和预期结果

**子代理纪律**：

- 写代码的子代理：读附近文件匹配风格 + 写完后自查函数问题
- 测试的子代理：给具体验证命令和预期结果
- 禁止子代理做未授权操作（删除文件、提交代码、修改非授权文件）

Migration/path policy:

- Canonical robot dog paths are under `/home/astra/works/robot_dog`; do not reintroduce old paths such as `/home/astra/works/UPRE_Dog`, `/home/astra/works/STM32`, `/home/astra/works/rl_training`, `/home/astra/rl_games_cache`, or `/home/astra/project/Dog`.
- Canonical AI tooling paths are under `/home/astra/works/ai`; do not reintroduce `/home/astra/AI_Coding` or `/home/astra/AI_chat`.
- Canonical robotics workspace paths are under `/home/astra/works/robotics`; do not reintroduce top-level `livox_ros2_ws`, `livox_ws`, `ws_livox`, or `slam_ws`.
- Canonical virtual environment paths are under `/home/astra/envs`, except `env_isaaclab`, which intentionally remains at `/home/astra/env_isaaclab`.
- Canonical data paths are under `/home/astra/data`; do not reintroduce top-level `livox_bag`.
- If updating scripts, README files, VS Code configs, launch files, or training configs, rewrite old absolute paths to the canonical paths above.
- Small arrow overlays in the file manager usually mean symlinks. Symlinks inside virtual environments, Snap directories, or `works/IsaacLab/_isaac_sim` are normal and should not be removed for visual cleanup.
