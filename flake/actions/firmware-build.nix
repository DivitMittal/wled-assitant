## Full PlatformIO build against pinned upstream WLED. This is the only place the usermod's
## static_asserts and the build-flag wiring get exercised, so it runs on every firmware change.
{common-actions, ...}: let
  paths = [
    "usermods/**"
    "platformio_override.ini"
    "scripts/**"
    "flake.lock"
    "flake/devshells.nix"
    "flake/actions/firmware-build.nix"
  ];
in {
  flake.actions-nix.workflows.".github/workflows/firmware-build.yml" = {
    on = {
      push = {
        branches = ["main"];
        inherit paths;
      };
      pull_request = {inherit paths;};
      workflow_dispatch = {};
    };
    jobs.building-firmware = {
      permissions.contents = "read";
      timeout-minutes = 60;
      steps =
        common-actions
        ++ [
          {
            ## nixpkgs' platformio is FHS-wrapped with bubblewrap, which
            ## Ubuntu 24.04 runners block via AppArmor's userns restriction.
            name = "Allow unprivileged user namespaces";
            run = "sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0";
          }
          {
            name = "Cache PlatformIO packages";
            uses = "actions/cache@v4";
            "with" = {
              path = "~/.platformio";
              key = "pio-\${{ runner.os }}-\${{ hashFiles('platformio_override.ini', 'scripts/bootstrap.sh') }}";
              restore-keys = "pio-\${{ runner.os }}-";
            };
          }
          {
            name = "Build firmware";
            run = "nix develop --impure -c scripts/build.sh";
          }
          {
            name = "Upload firmware";
            uses = "actions/upload-artifact@v4";
            "with" = {
              name = "firmware";
              path = "wled/build_output/release/*.bin";
              if-no-files-found = "error";
            };
          }
        ];
    };
  };
}
