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
            ;
          python = pkgs.python3.withPackages (ps: [ps.pyserial]); # tools/adalight_test.py
          ## AI context
          apm = customLib.mkUvxBin pkgs "apm" "--from apm-cli apm";
        };
      };
    };
  };
}
