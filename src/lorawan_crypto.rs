use core::{cell::RefCell, convert::TryInto};

use stm32wl_hal::aes::Aes;

use cipher::{BlockCipher, BlockDecrypt, BlockEncrypt, NewBlockCipher};
use cmac::{Cmac, Mac, NewMac};
use generic_array::{typenum::*, GenericArray};
use lorawan_encoding::keys::{CryptoFactory, Decrypter, Encrypter, Mac as LoraMac, AES128};

pub struct EncrypterDecrypter {
    aes: RefCell<Aes>,
    key: [u32; 4],
}

impl EncrypterDecrypter {
    pub fn new(key: &[u8; 16]) -> EncrypterDecrypter {
        let mut aes = unsafe { Aes::steal() };
        aes.set_dataswap(stm32wl_hal::aes::SwapMode::BYTE);
        let key_u32: [u32; 4] = [
            u32::from_be_bytes(key[0..4].try_into().unwrap()),
            u32::from_be_bytes(key[4..8].try_into().unwrap()),
            u32::from_be_bytes(key[8..12].try_into().unwrap()),
            u32::from_be_bytes(key[12..].try_into().unwrap()),
        ];
        EncrypterDecrypter {
            aes: RefCell::new(aes),
            key: key_u32,
        }
    }
}

impl Encrypter for EncrypterDecrypter {
    fn encrypt_block(&self, block: &mut GenericArray<u8, U16>) {
        let block_slice = block.as_mut_slice();
        // `block` might not be u32 aligned, so we cannot just use `block.as_mut_slice.align_to_mut::<u32>()`
        let mut cryptoblock: [u32; 4] = [
            u32::from_le_bytes(block_slice[0..4].try_into().unwrap()),
            u32::from_le_bytes(block_slice[4..8].try_into().unwrap()),
            u32::from_le_bytes(block_slice[8..12].try_into().unwrap()),
            u32::from_le_bytes(block_slice[12..].try_into().unwrap()),
        ];
        self.aes
            .borrow_mut()
            .encrypt_ecb_inplace(&self.key, &mut cryptoblock)
            .unwrap();
        block_slice.copy_from_slice(unsafe { &cryptoblock.align_to::<u8>().1 });
    }
}

impl Decrypter for EncrypterDecrypter {
    fn decrypt_block(&self, block: &mut GenericArray<u8, U16>) {
        let block_slice = block.as_mut_slice();
        // `block` might not be u32 aligned, so we cannot just use `block.as_mut_slice.align_to_mut::<u32>()`
        let mut cryptoblock: [u32; 4] = [
            u32::from_le_bytes(block_slice[0..4].try_into().unwrap()),
            u32::from_le_bytes(block_slice[4..8].try_into().unwrap()),
            u32::from_le_bytes(block_slice[8..12].try_into().unwrap()),
            u32::from_le_bytes(block_slice[12..].try_into().unwrap()),
        ];
        self.aes
            .borrow_mut()
            .encrypt_ecb_inplace(&self.key, &mut cryptoblock)
            .unwrap();
        block_slice.copy_from_slice(unsafe { &cryptoblock.align_to::<u8>().1 });
    }
}

pub struct CmacWl {
    cmac: Cmac<AesWl>,
}

impl CmacWl {
    pub fn new(key: &[u8; 16]) -> CmacWl {
        let cmac = Cmac::<AesWl>::new_from_slice(key).unwrap();
        CmacWl { cmac }
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

pub struct AesWl {
    aes: RefCell<Aes>,
    key: [u32; 4],
}

impl BlockCipher for AesWl {
    type BlockSize = U16;
    type ParBlocks = U0;
}

impl BlockEncrypt for AesWl {
    fn encrypt_block(&self, block: &mut cipher::Block<Self>) {
        let (_, plaintext, _) = unsafe { block.as_mut_slice().align_to_mut::<u32>() };
        self.aes
            .borrow_mut()
            .encrypt_ecb_inplace(&self.key, plaintext.try_into().unwrap())
            .unwrap();
    }
}

impl BlockDecrypt for AesWl {
    fn decrypt_block(&self, block: &mut cipher::Block<Self>) {
        let (_, plaintext, _) = unsafe { block.as_mut_slice().align_to_mut::<u32>() };
        self.aes
            .borrow_mut()
            .decrypt_ecb_inplace(&self.key, plaintext.try_into().unwrap())
            .unwrap();
    }
}

impl Clone for AesWl {
    /// While the implementation is necessary for Cmac because of the crypto traits,
    /// it is not used within the implementation. The Clone boundary is removed in the next
    /// version of the crypto traits.
    fn clone(&self) -> Self {
        Self {
            aes: unsafe { RefCell::new(Aes::steal()) },
            key: self.key.clone(),
        }
    }
}

impl NewBlockCipher for AesWl {
    type KeySize = U16;

    fn new(key: &GenericArray<u8, U16>) -> Self {
        let mut aes = unsafe { Aes::steal() };
        aes.set_dataswap(stm32wl_hal::aes::SwapMode::BYTE);
        let key_u32: [u32; 4] = [
            u32::from_be_bytes(key[0..4].try_into().unwrap()),
            u32::from_be_bytes(key[4..8].try_into().unwrap()),
            u32::from_be_bytes(key[8..12].try_into().unwrap()),
            u32::from_be_bytes(key[12..].try_into().unwrap()),
        ];
        AesWl {
            aes: RefCell::new(aes),
            key: key_u32,
        }
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
