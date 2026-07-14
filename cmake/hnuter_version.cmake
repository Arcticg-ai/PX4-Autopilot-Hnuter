# The Hnuter repository was imported from PX4 main during the v1.17
# development cycle without the upstream Git history and release tags.
#
# PX4 version format:
#   v<PX4 base version>-<vendor version>
#
# Keep the development suffix until this derivative is intentionally released.
set(HNUTER_PX4_BASE_VERSION "v1.17.0")
set(HNUTER_VENDOR_VERSION "1.0.0-dev")
set(HNUTER_FIRMWARE_VERSION
	"${HNUTER_PX4_BASE_VERSION}-${HNUTER_VENDOR_VERSION}")
