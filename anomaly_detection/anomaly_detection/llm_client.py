"""
LLM client and efficient image preparation for anomaly detection.

Supports LiteLLM integration (requires network connectivity and API keys) and
local model integration through Ollama.

Author: AAD Team Spring 26'
Version: 7/20/26
"""

import base64
from dataclasses import dataclass, field
import hashlib
from io import BytesIO
import os
from typing import Any, Iterable, List, Optional, Union

from dotenv import load_dotenv
import litellm
from ollama import Client
from PIL import Image
import yaml


DEFAULT_IMAGE_MAX_DIMENSION = 640
DEFAULT_IMAGE_JPEG_QUALITY = 75
DEFAULT_INFERENCE_TIMEOUT_SECONDS = 30.0
DEFAULT_KEEP_ALIVE = '2m'


load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))


@dataclass(frozen=True)
class PreparedImage:
    """A resized, JPEG-encoded image ready for reuse by an LLM backend."""

    data: bytes
    width: int
    height: int
    mime_type: str = 'image/jpeg'
    original_width: Optional[int] = None
    original_height: Optional[int] = None
    byte_size: int = field(init=False)
    sha256: str = field(init=False)

    def __post_init__(self) -> None:
        """Normalize immutable bytes and cache metadata used by artifacts."""
        encoded_bytes = bytes(self.data)
        object.__setattr__(self, 'data', encoded_bytes)
        object.__setattr__(self, 'byte_size', len(encoded_bytes))
        object.__setattr__(
            self,
            'sha256',
            hashlib.sha256(encoded_bytes).hexdigest(),
        )
        if self.original_width is None:
            object.__setattr__(self, 'original_width', self.width)
        if self.original_height is None:
            object.__setattr__(self, 'original_height', self.height)

    def to_base64(self) -> str:
        """Return the prepared bytes as base64 without re-encoding the image."""
        return base64.b64encode(self.data).decode('ascii')

    def to_data_url(self) -> str:
        """Return a data URL suitable for a remote multimodal API."""
        return f'data:{self.mime_type};base64,{self.to_base64()}'


def _positive_int(value: Any, default: Optional[int]) -> Optional[int]:
    """Return a positive integer config value, or the supplied default."""
    if value is None or isinstance(value, bool):
        return default

    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default

    return parsed if parsed > 0 else default


def _positive_float(value: Any, default: Optional[float]) -> Optional[float]:
    """Return a positive float config value, or the supplied default."""
    if value is None or isinstance(value, bool):
        return default

    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default

    return parsed if parsed > 0 else default


def prepare_image(
    image: Any,
    max_dimension: int = DEFAULT_IMAGE_MAX_DIMENSION,
    jpeg_quality: int = DEFAULT_IMAGE_JPEG_QUALITY,
) -> PreparedImage:
    """
    Resize and JPEG-encode an image once.

    ``image`` may be a numpy image array, a Pillow image, or an existing
    :class:`PreparedImage`. Prepared images are returned unchanged, which lets
    the caller reuse the same encoded bytes for inference and artifacts.

    Parameters
    ----------
    image
        Image data to prepare.
    max_dimension
        Maximum width or height, preserving aspect ratio.
    jpeg_quality
        Pillow JPEG quality from 1 through 100.

    Returns
    -------
    PreparedImage
        The JPEG bytes and their post-resize dimensions.

    Raises
    ------
    ValueError
        If either image preparation setting is invalid.
    TypeError
        If the supplied object cannot be converted to an image.

    """
    if isinstance(image, PreparedImage):
        return image

    if isinstance(max_dimension, bool) or not isinstance(max_dimension, int):
        raise ValueError('max_dimension must be a positive integer')
    if max_dimension <= 0:
        raise ValueError('max_dimension must be a positive integer')
    if isinstance(jpeg_quality, bool) or not isinstance(jpeg_quality, int):
        raise ValueError('jpeg_quality must be an integer between 1 and 100')
    if not 1 <= jpeg_quality <= 100:
        raise ValueError('jpeg_quality must be an integer between 1 and 100')

    try:
        pil_image = image if isinstance(image, Image.Image) else Image.fromarray(image)
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise TypeError('image must be a numpy array or Pillow image') from exc

    original_width, original_height = pil_image.size
    longest_edge = max(original_width, original_height)
    if longest_edge > max_dimension:
        scale = max_dimension / longest_edge
        resized_size = (
            max(1, round(original_width * scale)),
            max(1, round(original_height * scale)),
        )
        pil_image = pil_image.resize(resized_size, Image.Resampling.LANCZOS)

    if pil_image.mode not in ('RGB', 'L'):
        pil_image = pil_image.convert('RGB')

    buffer = BytesIO()
    pil_image.save(buffer, format='JPEG', quality=jpeg_quality)
    return PreparedImage(
        data=buffer.getvalue(),
        width=pil_image.width,
        height=pil_image.height,
        original_width=original_width,
        original_height=original_height,
    )


def encode_image(
    image_tensor: Any,
    max_dimension: int = DEFAULT_IMAGE_MAX_DIMENSION,
    jpeg_quality: int = DEFAULT_IMAGE_JPEG_QUALITY,
) -> str:
    """
    Return base64 JPEG data for backward-compatible callers.

    Passing a :class:`PreparedImage` only performs the required base64
    conversion; it does not resize or JPEG-encode the image again.

    """
    prepared = prepare_image(image_tensor, max_dimension, jpeg_quality)
    return prepared.to_base64()


class LLMClient:
    """Contact a remote LiteLLM model or a local Ollama model."""

    def __init__(self, config_path: Optional[str] = None):
        """Initialize model, image, and inference settings from YAML."""
        self.provider = 'openai'
        self.model_name = 'gpt-4o'
        self.model = None
        self.api_base = None
        self.system_prompt = None
        self.image_max_dimension = DEFAULT_IMAGE_MAX_DIMENSION
        self.image_jpeg_quality = DEFAULT_IMAGE_JPEG_QUALITY
        self.num_ctx = None
        self.num_predict = None
        self.think: Union[bool, str] = False
        self.inference_timeout_seconds = DEFAULT_INFERENCE_TIMEOUT_SECONDS
        self.keep_alive: Union[str, float, None] = DEFAULT_KEEP_ALIVE

        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        elif not os.path.isabs(config_path):
            config_path = os.path.abspath(config_path)

        llm_cfg = {}
        if os.path.isfile(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as config_file:
                    data = yaml.safe_load(config_file) or {}

                if isinstance(data, dict):
                    candidate = data.get('llm', {})
                    if isinstance(candidate, dict):
                        llm_cfg = candidate
                    self.provider = llm_cfg.get('model_provider', self.provider)
                    self.model_name = llm_cfg.get('model', self.model_name)
                    self.system_prompt = llm_cfg.get(
                        'system_prompt', self.system_prompt
                    )
                else:
                    print(
                        'Config file loaded but is not a YAML mapping. '
                        'Using defaults.'
                    )
            except Exception as exc:
                print(f'Failed to load config: {exc}. Using defaults.')
        else:
            print('Config file not found. Using defaults.')

        self._load_performance_config(llm_cfg)
        self.model = f'{self.provider}/{self.model_name}'
        self.api_base = os.getenv(f'{self.provider.upper()}_API_BASE', None)
        self.ollama_host = str(
            llm_cfg.get('ollama_host', 'http://localhost:11434')
        ).rstrip('/')

        ollama_options = {}
        if self.inference_timeout_seconds is not None:
            ollama_options['timeout'] = self.inference_timeout_seconds
        self.ollama_client = Client(
            host=self.ollama_host,
            **ollama_options,
        )

    def _load_performance_config(self, llm_cfg: dict) -> None:
        """Load optional image and inference tuning values."""
        image_cfg = llm_cfg.get('image', {})
        if not isinstance(image_cfg, dict):
            image_cfg = {}
        inference_cfg = llm_cfg.get('inference', {})
        if not isinstance(inference_cfg, dict):
            inference_cfg = {}

        image_max_dimension = llm_cfg.get(
            'image_max_dimension',
            image_cfg.get('max_dimension'),
        )
        image_quality = llm_cfg.get(
            'image_jpeg_quality',
            llm_cfg.get(
                'image_quality',
                image_cfg.get('jpeg_quality', image_cfg.get('quality')),
            ),
        )
        self.image_max_dimension = _positive_int(
            image_max_dimension,
            DEFAULT_IMAGE_MAX_DIMENSION,
        )
        self.image_jpeg_quality = _positive_int(
            image_quality,
            DEFAULT_IMAGE_JPEG_QUALITY,
        )
        if self.image_jpeg_quality > 100:
            self.image_jpeg_quality = DEFAULT_IMAGE_JPEG_QUALITY

        self.num_ctx = _positive_int(
            llm_cfg.get('num_ctx', inference_cfg.get('num_ctx')),
            None,
        )
        self.num_predict = _positive_int(
            llm_cfg.get('num_predict', inference_cfg.get('num_predict')),
            None,
        )
        think_value = llm_cfg.get('think', inference_cfg.get('think', False))
        if isinstance(think_value, bool) or think_value in (
            'low',
            'medium',
            'high',
        ):
            self.think = think_value
        timeout_value = llm_cfg.get(
            'timeout_seconds',
            llm_cfg.get(
                'inference_timeout_seconds',
                llm_cfg.get(
                    'timeout',
                    inference_cfg.get(
                        'timeout_seconds',
                        inference_cfg.get('timeout'),
                    ),
                ),
            ),
        )
        self.inference_timeout_seconds = _positive_float(
            timeout_value,
            DEFAULT_INFERENCE_TIMEOUT_SECONDS,
        )

        keep_alive = llm_cfg.get(
            'keep_alive',
            inference_cfg.get('keep_alive', DEFAULT_KEEP_ALIVE),
        )
        if keep_alive is None or isinstance(keep_alive, (str, int, float)):
            self.keep_alive = keep_alive

    def prepare_images(
        self,
        images: Optional[Iterable[Any]] = None,
    ) -> List[PreparedImage]:
        """Prepare raw images, preserving already-prepared inputs."""
        if images is None:
            return []
        return [
            prepare_image(
                image,
                max_dimension=self.image_max_dimension,
                jpeg_quality=self.image_jpeg_quality,
            )
            for image in images
        ]

    def chat(
        self,
        text: str,
        images: Optional[Iterable[Any]] = None,
    ) -> str:
        """Call a remote LiteLLM model with text and optional images."""
        content = [{'type': 'text', 'text': text}]
        prepared_images = self.prepare_images(images)

        for image in prepared_images:
            content.append(
                {
                    'type': 'image_url',
                    'image_url': {'url': image.to_data_url()},
                }
            )

        completion_options = {
            'model': self.model,
            'messages': [
                {'role': 'system', 'content': self.system_prompt},
                {'role': 'user', 'content': content},
            ],
            'api_base': self.api_base,
        }
        if self.inference_timeout_seconds is not None:
            completion_options['timeout'] = self.inference_timeout_seconds
        if self.num_predict is not None:
            completion_options['max_tokens'] = self.num_predict

        response = litellm.completion(**completion_options)
        return response.choices[0].message.content

    def local_chat(
        self,
        text: str,
        images: Optional[Iterable[Any]] = None,
    ) -> str:
        """Call a local Ollama model with text and optional images."""
        messages = []

        if self.system_prompt:
            messages.append({'role': 'system', 'content': self.system_prompt})

        user_message = {'role': 'user', 'content': text}
        prepared_images = self.prepare_images(images)
        if prepared_images:
            user_message['images'] = [image.data for image in prepared_images]

        messages.append(user_message)

        options = {}
        if self.num_ctx is not None:
            options['num_ctx'] = self.num_ctx
        if self.num_predict is not None:
            options['num_predict'] = self.num_predict

        chat_options = {
            'model': self.model_name,
            'messages': messages,
            'stream': False,
            'think': self.think,
            'format': 'json',
            'keep_alive': self.keep_alive,
        }
        if options:
            chat_options['options'] = options

        response = self.ollama_client.chat(**chat_options)
        return response.message.content


def main() -> None:
    """Run a small text-only client smoke test."""
    client = LLMClient()
    print('=== Text-only test ===')
    response = client.chat(
        'Motor encoder error: velocity mismatch detected at joint 3.'
    )
    print(response)


if __name__ == '__main__':
    main()
