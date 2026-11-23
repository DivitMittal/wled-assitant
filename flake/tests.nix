## Host-side test suites, built by `nix flake check` (and therefore by CI).
{lib, ...}: {
  perSystem = {pkgs, ...}: let
    src = lib.fileset.toSource {
      root = ../.;
      fileset = lib.fileset.unions [
        ../tests
        ../tools
        ../platformio_override.ini
      ];
    };
    python = pkgs.python3.withPackages (ps: [ps.pytest]);
  in {
    checks.pytest = pkgs.runCommand "wled-assitant-pytest" {nativeBuildInputs = [python];} ''
      cd ${src}
      pytest -p no:cacheprovider -v tests
      touch $out
    '';
  };
}
