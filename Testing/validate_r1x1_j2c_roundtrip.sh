#!/usr/bin/env bash
set -euo pipefail

conda_env_name="mcam_dev"
num_decomps="${NUM_DECOMPS:-5}"
kdu_expand_executable="${KDU_EXPAND:-kdu_expand}"
kdu_compress_executable="${KDU_COMPRESS:-kdu_compress}"
work_dir="${TMPDIR:-/tmp}/openjph_r1x1_validate_$$"
canvas_size=512
circle_radius=150
input_pgm="${work_dir}/circle.pgm"
openjph_j2c="${work_dir}/openjph_r1x1.j2c"
decoded_by_kdu_pgm="${work_dir}/decoded_by_kdu.pgm"
kakadu_ht_r1x1_j2c="${work_dir}/kakadu_ht_r1x1.j2c"
decoded_by_ojph_pgm="${work_dir}/decoded_by_ojph.pgm"

set +u
eval "$(conda shell.bash hook)"
conda activate "${conda_env_name}"
set -u

if ! command -v "${kdu_expand_executable}" >/dev/null 2>&1; then
  echo "kdu_expand not found (tried: ${kdu_expand_executable})." >&2
  echo "Set KDU_EXPAND to the full path, e.g. KDU_EXPAND=\$HOME/bin/kdu_expand" >&2
  exit 1
fi

if ! command -v "${kdu_compress_executable}" >/dev/null 2>&1; then
  echo "kdu_compress not found (tried: ${kdu_compress_executable})." >&2
  echo "Set KDU_COMPRESS to the full path, e.g. KDU_COMPRESS=\$HOME/bin/kdu_compress" >&2
  exit 1
fi

mkdir -p "${work_dir}"

LC_ALL=C awk -v w="${canvas_size}" -v h="${canvas_size}" -v r="${circle_radius}" 'BEGIN {
  cx = int(w / 2)
  cy = int(h / 2)
  rs = r * r
  printf "P5\n%d %d\n255\n", w, h
  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++) {
      d = (x - cx) * (x - cx) + (y - cy) * (y - cy)
      printf "%c", (d <= rs) ? 255 : 0
    }
  }
}' > "${input_pgm}"

ojph_compress -i "${input_pgm}" -o "${openjph_j2c}" -reversible true -r1x1 true \
  -num_decomps "${num_decomps}"
"${kdu_expand_executable}" -i "${openjph_j2c}" -o "${decoded_by_kdu_pgm}"

if ! cmp -s "${input_pgm}" "${decoded_by_kdu_pgm}"; then
  echo "phase 1 failed: OpenJPH encode + Kakadu expand, PGM mismatch" >&2
  exit 1
fi
echo "phase 1 ok: OpenJPH compress + Kakadu expand, PGM matches"

"${kdu_compress_executable}" -i "${input_pgm}" -o "${kakadu_ht_r1x1_j2c}" \
  Creversible=yes Kkernels=R1X1 "Clevels=${num_decomps}" Cmodes=HT

ojph_expand -i "${kakadu_ht_r1x1_j2c}" -o "${decoded_by_ojph_pgm}"

if ! cmp -s "${input_pgm}" "${decoded_by_ojph_pgm}"; then
  echo "phase 2 failed: Kakadu HT R1X1 encode + OpenJPH expand, PGM mismatch" >&2
  exit 1
fi
echo "phase 2 ok: Kakadu compress (Kkernels=R1X1 Cmodes=HT) + OpenJPH expand, PGM matches"

echo "artifacts: ${work_dir}"
