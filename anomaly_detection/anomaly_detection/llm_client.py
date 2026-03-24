"""
General-purpose LiteLLM client with .env + config.yaml support.

Author: Allie O'Keeffe
Version: 3/3/2026
"""
import os
import litellm
import yaml
from io import BytesIO
from PIL import Image
import base64
from dotenv import load_dotenv

# Load .env automatically
load_dotenv()

def encode_image(image_tensor):
    """
    Encode a numpy image array as base64 PNG string
    """
    image = Image.fromarray(image_tensor)
    buffered = BytesIO()
    image.save(buffered, format="PNG")
    return base64.b64encode(buffered.getvalue()).decode("utf-8")


class LLMClient:
    """
    Description
    ------------
        General-purpose LiteLLM client that loads config from config.yaml and API keys from environment variables. Supports sending text and images to the LLM.
    
    Attributes
    ----------
        provider (str): The LLM provider name.

        model_name (str): The specific model name.

        model (str): The composed LiteLLM model string.

        api_key (str): The API key for the LLM provider.
        
        api_base (str): The base URL for the LLM API.

    Methods
    -------
        chat(text: str, images=None):
            Send a message to specified LLM, find proper API key for auth.
    
    """

    ## TODO this path may not be valid b/c of the context being of working dir and not script, self.provider is never created and we fail looking for an attr that doesnt exist at line 63.
    def __init__(self, config_path="config.yaml"):
        # Load YAML config
        config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
        
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
            self.provider = cfg.get("llm", {}).get("provider", self.provider)
            self.model_name = cfg.get("llm", {}).get("model", self.model_name)

        # Compose LiteLLM model string
        self.model = f"{self.provider}/{self.model_name}"

        # Load env vars
        self.api_key = os.getenv(f"{self.provider.upper()}_API_KEY")
        self.api_base = os.getenv(f"{self.provider.upper()}_API_BASE", None)

    def chat(self, text, images=None):
        """
        Send a message to the LLM, optionally with images

        Args:
            text (str): user prompt
            images (list of np.ndarray): optional images

        Returns:
            str: LLM response
        """
        content = [{"type": "text", "text": text}]
        if images:
            for img in images:
                img_b64 = encode_image(img)
                content.append(
                    {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{img_b64}"}}
                )

        ## TODO are api_key and api_base used in all completions (openai/anthropic)? Also, would this work for AWS bedrock (AWS_ACCESS_KEY_ID)
        ## TODO also lets verify that response is not just NULL or something when the API key is bad
        response = litellm.completion(
            model=self.model,
            messages=[{"role": "user", "content": content}],
            api_key=self.api_key,
            api_base=self.api_base,
        )

        return response["choices"][0]["message"]["content"]
