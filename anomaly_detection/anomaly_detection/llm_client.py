import os
import litellm
import yaml
from io import BytesIO
from PIL import Image
import base64
from ollama import Client
from dotenv import load_dotenv

load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))

def encode_image(image_tensor):
    """
    Encode a numpy image array as base64 PNG string.
    """
    image = Image.fromarray(image_tensor)
    buffered = BytesIO()
    image.save(buffered, format="PNG")
    return base64.b64encode(buffered.getvalue()).decode("utf-8")


class LLMClient:
    def __init__(self, config_path=None):
        # Always define attributes first
        self.provider = "openai"
        self.model_name = "gpt-4o"
        self.model = None
        self.api_base = None
        self.system_prompt = None

        # Match the same config resolution style as anomaly_detection_node.py
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")
        elif not os.path.isabs(config_path):
            config_path = os.path.abspath(config_path)

        if os.path.isfile(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}

                if isinstance(data, dict):
                    llm_cfg = data.get("llm", {})
                    self.provider = llm_cfg.get("model_provider", self.provider)
                    self.model_name = llm_cfg.get("model", self.model_name)
                    self.system_prompt = llm_cfg.get("system_prompt", self.system_prompt)
                else:
                    print("Config file loaded but is not a YAML mapping. Using defaults.")
            except Exception as e:
                print(f"Failed to load config: {e}. Using defaults.")
        else:
            print("Config file not found. Using defaults.")

        self.model = f"{self.provider}/{self.model_name}"
        self.api_base = os.getenv(f"{self.provider.upper()}_API_BASE", None)
        
    def chat(self, text, images=None):
        content = [{"type": "text", "text": text}]

        if images:
            for img in images:
                img_b64 = encode_image(img)
                content.append(
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/png;base64,{img_b64}"
                        },
                    }
                )

        litellm._turn_on_debug()
        response = litellm.completion(
            model=self.model,
            messages =  [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": content}  # <-- your log payload goes here
            ],
            api_base=self.api_base,
        )

        return response.choices[0].message.content

    def local_chat(self, text, images=None):
        """
        Ping an Ollama hosted model given some data.
        
        Args
        ----
            text (str): text data

            images (list): list of numpy image arrays 
        
        Returns
        -------
            str: the models response
        
        """
        messages = []

        if self.system_prompt:
            messages.append({"role": "system", "content": self.system_prompt})

        user_message = {"role": "user", "content": text}
        if images:
            user_message["images"] = [encode_image(img) for img in images]

        messages.append(user_message)

        client = Client(host="http://localhost:11434")
        response = client.chat(
            model=self.model_name,
            messages=messages,
            stream=False,
            format="json",
            keep_alive="30m",
        )

        return response.message.content
    
def main():
    client = LLMClient()

    # Test 1: text only
    print("=== Text-only test ===")
    response = client.chat("Motor encoder error: velocity mismatch detected at joint 3.")
    print(response)

    # Test 2: with a dummy image (random noise as stand-in for a real frame)
    print("\n=== Image + text test ===")
    # dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    # response = client.chat(
    #     text="Analyze this frame for anomalies. No errors in the log.",
    #     images=[dummy_image],
    # )
    # print(response)

if __name__ == "__main__":
    main()
