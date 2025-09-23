docker run --runtime nvidia -it -v ${HOME}/model_data:/home/developer/model_data -v ${HOME}/javis_ws:/home/developer/javis_ws --network=host dustynv/kokoro-tts:fastapi-r36.4.0
