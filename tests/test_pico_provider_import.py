from teleop.input.pico_provider import PicoProvider


def test_pico_provider_import_and_instantiate() -> None:
    provider = PicoProvider()
    assert provider is not None
    assert provider.get_latest() is None
