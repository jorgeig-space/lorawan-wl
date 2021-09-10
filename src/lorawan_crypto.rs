use aes::Aes128;
use aes::cipher::{
    BlockEncrypt, BlockDecrypt, NewBlockCipher,
    generic_array::GenericArray,
    generic_array::typenum::*,
};

use lorawan_encoding::keys::{AES128, CryptoFactory, Decrypter, Encrypter, Mac as LoraMac};
use cmac::{Cmac, NewMac, Mac};

pub struct EncrypterDecrypter {
    cipher: Aes128,
}

impl EncrypterDecrypter {
    pub fn new(key: &[u8; 16]) -> EncrypterDecrypter {
        EncrypterDecrypter { cipher:  Aes128::new_from_slice(key).unwrap(), }
    }
}

impl Encrypter for EncrypterDecrypter {
    fn encrypt_block(&self, block: &mut GenericArray<u8, U16>) {
        self.cipher.encrypt_block(block)
    }
}

impl Decrypter for EncrypterDecrypter {
    fn decrypt_block(&self, block: &mut GenericArray<u8, U16>) {
        self.cipher.decrypt_block(block)
    }
}

pub struct CmacWl {
    cmac: Cmac<Aes128>,
}

impl CmacWl {
    pub fn new(key: &[u8; 16]) -> CmacWl {
        let cmac = Cmac::<Aes128>::new_from_slice(key).unwrap();
        CmacWl {cmac}
    }
}

impl LoraMac for CmacWl {
    fn input(&mut self, data: &[u8]) {
        self.cmac.update(data)
    }

    fn reset(&mut self) {
        self.cmac.reset()
    }

    fn result(self) -> GenericArray<u8, U16> {
        self.cmac.finalize().into_bytes()
    }
}

#[derive(Default)]
pub struct LorawanCrypto;

impl CryptoFactory for LorawanCrypto {
    type E = EncrypterDecrypter;
    type D = EncrypterDecrypter;
    type M = CmacWl;

    fn new_enc(&self, key: &AES128) -> Self::E {
        EncrypterDecrypter::new(&key.0)
    }

    fn new_dec(&self, key: &AES128) -> Self::D {
        EncrypterDecrypter::new(&key.0)
    }

    fn new_mac(&self, key: &AES128) -> Self::M {
        CmacWl::new(&key.0)
    }
}