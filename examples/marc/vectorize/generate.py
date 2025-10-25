"""Text-to-image utilities built on top of the Diffusers library."""
from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Optional

import torch

try:
    from diffusers import StableDiffusionPipeline, StableDiffusionXLPipeline
except ImportError as exc:  # pragma: no cover - only triggered when diffusers is missing.
    raise RuntimeError(
        "The diffusers package is required for examples.marc.vectorize.generate"
    ) from exc


LOGGER = logging.getLogger(__name__)


def _default_device() -> str:
    """Return the most capable available device."""
    if torch.cuda.is_available():
        return "cuda"
    if hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def load_text2img_pipeline(
    preferred_model: str = "stabilityai/stable-diffusion-xl-base-1.0",
    fallback_model: str = "runwayml/stable-diffusion-v1-5",
    *,
    device: Optional[str] = None,
    torch_dtype: Optional[torch.dtype] = None,
):
    """Load a text-to-image diffusion pipeline.

    Parameters
    ----------
    preferred_model:
        Hugging Face model identifier to attempt to load first (SDXL by default).
    fallback_model:
        Model identifier to load if the preferred model cannot be instantiated.
    device:
        Optional device string. If ``None`` we will select ``cuda`` if available
        then ``mps`` then ``cpu``.
    torch_dtype:
        Optional dtype hint. ``float16`` is recommended for GPU devices whereas
        ``float32`` is recommended on CPU devices.
    """

    target_device = device or _default_device()
    if torch_dtype is None:
        torch_dtype = torch.float16 if target_device != "cpu" else torch.float32

    def _attempt_load(model_id: str, cls):
        LOGGER.info("Loading pipeline %s on %s", model_id, target_device)
        pipe = cls.from_pretrained(model_id, torch_dtype=torch_dtype)
        pipe.to(target_device)
        return pipe

    try:
        return _attempt_load(preferred_model, StableDiffusionXLPipeline)
    except Exception as preferred_error:  # pragma: no cover - depends on runtime env.
        LOGGER.warning("Falling back to SD 1.5 pipeline: %s", preferred_error)
        pipe = _attempt_load(fallback_model, StableDiffusionPipeline)
        return pipe


def generate_image(
    prompt: str,
    output_path: Path,
    *,
    model_id: Optional[str] = None,
    fallback_model_id: str = "runwayml/stable-diffusion-v1-5",
    negative_prompt: Optional[str] = None,
    num_inference_steps: int = 30,
    guidance_scale: float = 7.5,
    height: Optional[int] = None,
    width: Optional[int] = None,
    seed: Optional[int] = None,
    device: Optional[str] = None,
    torch_dtype: Optional[torch.dtype] = None,
) -> Path:
    """Generate an image from a text prompt and save it as a PNG.

    Parameters are intentionally similar to the diffusers pipeline ``__call__``
    signature to make it easy to experiment with inference behaviour.
    """

    pipeline = load_text2img_pipeline(
        preferred_model=model_id or "stabilityai/stable-diffusion-xl-base-1.0",
        fallback_model=fallback_model_id,
        device=device,
        torch_dtype=torch_dtype,
    )

    generator = None
    if seed is not None:
        generator_device = pipeline.device
        if isinstance(generator_device, torch.device):
            generator_device = generator_device.type
        generator = torch.Generator(device=generator_device).manual_seed(seed)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    LOGGER.info("Generating image for prompt: %s", prompt)
    result = pipeline(
        prompt,
        negative_prompt=negative_prompt,
        num_inference_steps=num_inference_steps,
        guidance_scale=guidance_scale,
        height=height,
        width=width,
        generator=generator,
    )
    image = result.images[0]
    image.save(output_path)
    LOGGER.info("Saved generated image to %s", output_path)
    return output_path


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a PNG image from a text prompt")
    parser.add_argument("prompt", help="Text prompt to feed into the diffusion model")
    parser.add_argument("output", help="Destination path for the generated PNG")
    parser.add_argument("--model", dest="model_id", default=None, help="Preferred model id")
    parser.add_argument(
        "--fallback-model",
        dest="fallback_model_id",
        default="runwayml/stable-diffusion-v1-5",
        help="Fallback model identifier if the preferred model cannot be loaded",
    )
    parser.add_argument("--negative", dest="negative_prompt", default=None, help="Negative prompt")
    parser.add_argument("--steps", dest="num_inference_steps", type=int, default=30)
    parser.add_argument("--scale", dest="guidance_scale", type=float, default=7.5)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument(
        "--dtype",
        default=None,
        choices=["float16", "float32"],
        help="Torch dtype to use (overrides automatic selection)",
    )
    return parser


def _parse_dtype(dtype_str: Optional[str]) -> Optional[torch.dtype]:
    if dtype_str is None:
        return None
    return torch.float16 if dtype_str == "float16" else torch.float32


def main(argv: Optional[list[str]] = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    dtype = _parse_dtype(args.dtype)
    output_path = Path(args.output)

    generate_image(
        prompt=args.prompt,
        output_path=output_path,
        model_id=args.model_id,
        fallback_model_id=args.fallback_model_id,
        negative_prompt=args.negative_prompt,
        num_inference_steps=args.num_inference_steps,
        guidance_scale=args.guidance_scale,
        height=args.height,
        width=args.width,
        seed=args.seed,
        device=args.device,
        torch_dtype=dtype,
    )
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
