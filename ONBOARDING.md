# Team setup guide (do once per machine)

Takes ~30 minutes. Shared team conventions live in `AGENTS.md`;
this file is only about getting *your* machine working.

## 1. Android Studio
- Install **Android Studio Narwhal 3 Feature Drop or newer**.
- Open the `ftc-main` project folder, let Gradle sync finish
  (first sync downloads a lot — be patient).
- `local.properties` (your SDK path) is generated automatically and is
  git-ignored. Never commit it.

## 2. Git access
Clone with SSH (no passwords, works for terminals and bots):

```sh
git clone git@github.com:NomadicDragons/ftc2026.git
```

If that fails with a permission error, add your SSH key to GitHub once:
1. `ssh-keygen -t ed25519 -C "you@example.com"` (Enter = no passphrase is fine here)
2. Copy `~/.ssh/id_ed25519.pub` into GitHub → Settings → SSH and GPG keys → New SSH key
3. Retry the clone.

## 3. Terminal builds (JAVA_HOME)
Your system Java is probably too old for Gradle 9. Point terminal builds
at Android Studio's bundled Java:

- macOS (`~/.zshrc`):
  ```sh
  export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
  ```
- Windows: set `JAVA_HOME` to `C:\Program Files\Android\Android Studio\jbr`
  (System Properties → Environment Variables).

Then `cd ftc-main && ./gradlew assembleDebug` must succeed.

## 4. OpenCode + Android Studio link
1. Install OpenCode (see https://opencode.ai — follow the install guide).
2. In Android Studio: **Settings → Plugins → Marketplace**, search
   **"MCP Server"** (by JetBrains) and install it.
   - The plugin version **must match your Android Studio build**
     (Help → About shows e.g. `261.26222.65`). If the Marketplace only
     offers an incompatible version, download the matching one from
     https://plugins.jetbrains.com/plugin/26071-mcp-server/versions
     (match the `since-build`) and install via
     Settings → Plugins → ⚙ → Install Plugin from Disk.
   - Restart Android Studio.
3. **Settings → Tools → MCP Server → ☑ Enable MCP Server → Apply.**
   (This is *not* the Gemini "AI → MCP Servers" page — that one is for
   Gemini, and OpenCode doesn't use it.)
4. Find your MCP port — in the IDE log, look for a line like
   `Responding at http://127.0.0.1:64342`:
   - macOS: `~/Library/Logs/Google/AndroidStudio*/idea.log`
   - Windows: `%LOCALAPPDATA%\Google\AndroidStudio*\log\idea.log`
5. Register it with OpenCode (global = all your projects):
   ```sh
   opencode mcp add android-studio --global --url http://127.0.0.1:PORT/stream
   ```
6. Verify: `opencode mcp list` → `✓ android-studio connected`.
   If it says `failed`, the port changed or the IDE checkbox is off —
   re-check steps 3–4.

## 5. Daily workflow
- `git pull` when you start. Work in your personal branch.
- Done with a feature? Open a PR into **`staging`**.
- `staging` gets tested on the robot, then goes to **`main`** via PR.
- Never push directly to `main` or `staging`.
- Team rules are in `AGENTS.md` — read it once.
- Robot config XMLs: export from the Robot Controller and commit them
  under `TeamCode/configs/` so they're backed up.
