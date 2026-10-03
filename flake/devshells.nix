{
  inputs,
  lib,
  ...
}: let
  customLib = import (inputs.infra-nixCfg + "/lib/custom.nix") {inherit (inputs.nixpkgs) lib;};
in {
  imports = [inputs.devshell.flakeModule];

  perSystem = {
    pkgs,
    config,
    ...
  }: {
    devshells.default = {
      devshell = rec {
        name = "wled-assitant";
        motd = "{202}Welcome to {91}${name} {202}devshell!{reset} \n $(menu)";
        startup = {
          git-hooks.text = ''
            ${config.pre-commit.installationScript}
          '';
        };
        packages = lib.attrsets.attrValues {
          inherit
            (pkgs)
            ### LSPs & Formatters
            ## Nix
            nixd
            alejandra
            ## C++ (clangd for the usermod; needs `fw-compiledb`)
            clang-tools
            ### ESP32 firmware
            platformio # pio; FHS-wrapped on Linux so its prebuilt toolchains run
            esptool # standalone chip/flash info, erase, merge-bin, recovery flashing
            nodejs # WLED web UI build (npm ci / npm run build)
            tio # serial monitor that leaves DTR/RTS alone (no USB-JTAG reset)
            uv # `uv run --with pyserial tools/adalight_test.py`
            mosquitto # mosquitto_sub/pub for checking MQTT/HA discovery
            ### VCS
            jujutsu # colocated jj; local worktrees are jj workspaces under .worktrees/
            ;
          python = pkgs.python3.withPackages (ps: [ps.pyserial]); # tools/adalight_test.py
          ## AI context
          apm = customLib.mkUvxBin pkgs "apm" "--from apm-cli apm";
        };
      };
      commands = let
        env = "wled_assitant_s3_supermini";
        # root of the workspace we're in (main checkout or .worktrees/<name>), not the main checkout
        root = ''root="$(jj workspace root 2>/dev/null || echo "$PRJ_ROOT")"; '';
        # main checkout: jj workspaces have no .git, so git resolves to it from anywhere
        main = ''set -euo pipefail; main="$(git rev-parse --show-toplevel)"; '';
      in
        map (c: c // {category = "firmware";}) [
          {
            name = "fw-build";
            help = "bootstrap pinned WLED + build firmware";
            command = root + ''"$root/scripts/build.sh" "$@"'';
          }
          {
            name = "fw-flash";
            help = "build + flash over USB: fw-flash <port> (stop HyperHDR first)";
            command = root + ''"$root/scripts/build.sh" upload --upload-port "''${1:?usage: fw-flash <port>}"'';
          }
          {
            name = "fw-erase";
            help = "erase all flash incl. WLED settings: fw-erase <port>";
            command = root + ''cd "$root/wled" && pio run -e ${env} -t erase --upload-port "''${1:?usage: fw-erase <port>}"'';
          }
          {
            name = "fw-info";
            help = "chip, flash size and PSRAM check: fw-info <port>";
            command = ''esptool --chip esp32s3 --port "''${1:?usage: fw-info <port>}" flash-id'';
          }
          {
            name = "fw-monitor";
            help = "serial console (shares the HyperHDR port): fw-monitor <port>";
            command = ''tio "''${1:?usage: fw-monitor <port>}"'';
          }
          {
            name = "fw-ports";
            help = "list candidate USB serial ports";
            command = ''ls /dev/cu.usbmodem* /dev/ttyACM* 2>/dev/null || echo "no ESP32-S3 USB port found"'';
          }
          {
            name = "fw-compiledb";
            help = "generate wled/compile_commands.json for clangd/serena";
            command = root + ''cd "$root/wled" && pio run -e ${env} -t compiledb'';
          }
        ]
        ++ map (c: c // {category = "worktrees";}) [
          {
            name = "wt-add";
            help = "new local worktree (jj workspace) at .worktrees/<name>: wt-add <name> [revision]";
            command =
              main
              + ''
                name="''${1:?usage: wt-add <name> [revision]}"
                case "$name" in */* | .* | "") echo "invalid workspace name: $name" >&2; exit 1 ;; esac
                mkdir -p "$main/.worktrees"
                rev=(); if [ -n "''${2:-}" ]; then rev=(-r "$2"); fi
                jj -R "$main" workspace add --name "$name" "''${rev[@]}" "$main/.worktrees/$name"
                echo "next: cd .worktrees/$name && direnv allow && fw-build  (WLED is cloned from the main checkout)"
              '';
          }
          {
            name = "wt-list";
            help = "list jj workspaces";
            command = main + ''jj -R "$main" workspace list'';
          }
          {
            name = "wt-rm";
            help = "forget a worktree and delete its directory (its changes stay in jj): wt-rm <name>";
            command =
              main
              + ''
                name="''${1:?usage: wt-rm <name>}"
                dir="$main/.worktrees/$name"
                [ -d "$dir" ] || { echo "no worktree at $dir" >&2; exit 1; }
                case "$PWD/" in "$dir"/*) echo "leave $dir first" >&2; exit 1 ;; esac
                jj -R "$dir" status >/dev/null   # snapshot uncommitted edits into its working-copy change
                jj -R "$main" workspace forget "$name"
                rm -rf "$dir"
                echo "removed $dir; its last change is still in 'jj log'"
              '';
          }
        ];
    };
  };
}
