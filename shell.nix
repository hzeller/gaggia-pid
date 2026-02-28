# This is a nix-shell for use with the nix package manager.
# If you have nix installed, you may simply run `nix-shell`
# in this repo, and have all dependencies ready in the new shell.

{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  buildInputs = with pkgs;
    [
      kicad
      # text-size: gcc13->7920; gcc14->7900; gcc15->7942
      # Choosing 14 for that reason.
      pkgsCross.avr.buildPackages.gcc14
      avrdude
      tio
    ];
}
