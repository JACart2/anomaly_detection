"""Tests for efficient LLM image preparation and backend payloads."""

import base64
import hashlib
from io import BytesIO
import os
import sys
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from PIL import Image
import yaml

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from anomaly_detection.llm_client import (  # noqa: E402, I100
    encode_image,
    LLMClient,
    prepare_image,
    PreparedImage,
)


def test_prepare_image_resizes_aspect_ratio_and_encodes_jpeg_once():
    """The prepared representation contains a bounded, valid JPEG."""
    raw_image = np.zeros((600, 1200, 3), dtype=np.uint8)
    raw_image[:, :, 0] = 180

    prepared = prepare_image(raw_image)

    assert prepared.width == 640
    assert prepared.height == 320
    assert prepared.original_width == 1200
    assert prepared.original_height == 600
    assert prepared.mime_type == 'image/jpeg'
    assert prepared.byte_size == len(prepared.data)
    assert prepared.sha256 == hashlib.sha256(prepared.data).hexdigest()
    with Image.open(BytesIO(prepared.data)) as decoded:
        assert decoded.format == 'JPEG'
        assert decoded.size == (640, 320)

    # An existing prepared image crosses the same API boundary unchanged.
    assert prepare_image(prepared) is prepared
    assert base64.b64decode(encode_image(prepared)) == prepared.data


def test_prepare_image_does_not_upscale_small_frames():
    """Small images retain their original dimensions."""
    raw_image = np.zeros((120, 160, 3), dtype=np.uint8)

    prepared = prepare_image(raw_image, max_dimension=640, jpeg_quality=60)

    assert (prepared.width, prepared.height) == (160, 120)


def test_configured_prepared_image_payloads_for_ollama_and_litellm(tmp_path):
    """Both backends reuse JPEG bytes and receive their relevant limits."""
    config_path = tmp_path / 'config.yaml'
    config_path.write_text(
        """
llm:
  model_provider: gemma
  model: gemma4:12b-it-qat
  system_prompt: inspect the event
  image_max_dimension: 320
  image_jpeg_quality: 82
  num_ctx: 4096
  num_predict: 96
  timeout_seconds: 12.5
  keep_alive: 2m
""".lstrip(),
        encoding='utf-8',
    )
    jpeg_bytes = b'already-prepared-jpeg-bytes'
    prepared = PreparedImage(jpeg_bytes, width=320, height=180)

    with patch('anomaly_detection.llm_client.Client') as client_class:
        client = LLMClient(str(config_path))

    client_class.assert_called_once_with(
        host='http://localhost:11434',
        timeout=12.5,
    )
    assert client.image_max_dimension == 320
    assert client.image_jpeg_quality == 82
    assert client.prepare_images([prepared])[0] is prepared

    client.ollama_client.chat.return_value = SimpleNamespace(
        message=SimpleNamespace(content='{"anomaly": false}')
    )
    assert client.local_chat('event', images=[prepared]) == '{"anomaly": false}'
    local_call = client.ollama_client.chat.call_args.kwargs
    assert local_call['messages'][-1]['images'] == [jpeg_bytes]
    assert local_call['options'] == {'num_ctx': 4096, 'num_predict': 96}
    assert local_call['think'] is False
    assert local_call['keep_alive'] == '2m'

    remote_response = SimpleNamespace(
        choices=[SimpleNamespace(message=SimpleNamespace(content='remote result'))]
    )
    with patch(
        'anomaly_detection.llm_client.litellm.completion',
        return_value=remote_response,
    ) as completion:
        assert client.chat('event', images=[prepared]) == 'remote result'

    remote_call = completion.call_args.kwargs
    image_url = remote_call['messages'][1]['content'][1]['image_url']['url']
    assert image_url == (
        'data:image/jpeg;base64,' + base64.b64encode(jpeg_bytes).decode('ascii')
    )
    assert remote_call['max_tokens'] == 96
    assert remote_call['timeout'] == 12.5


def test_optional_image_arguments_are_not_mutable_defaults():
    """Public chat methods use ``None`` for optional image collections."""
    assert LLMClient.chat.__defaults__ == (None,)
    assert LLMClient.local_chat.__defaults__ == (None,)
    assert LLMClient.prepare_images.__defaults__ == (None,)


def test_default_runtime_config_enables_gemma4_vision():
    """The deployed default uses the intended local multimodal model."""
    config_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        'anomaly_detection',
        'config.yaml',
    )
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    with patch('anomaly_detection.llm_client.Client'):
        client = LLMClient(config_path)

    assert config['llm']['vision_enabled'] is True
    assert client.model_name == 'gemma4:12b-it-qat'
    assert client.think is False
