class DataStorage:

    def start_session(name: str | None = None):
        pass

    def end_session():
        pass

    def commit():
        pass

    def log(message: str):
        pass

    def log_error(message: str, exception: Exception | None = None):
        pass

    def save_image(image, filename: str, position):
        pass

    def save_csv(filename: str, rows: list, header: list[str] | None = None):
        pass



    