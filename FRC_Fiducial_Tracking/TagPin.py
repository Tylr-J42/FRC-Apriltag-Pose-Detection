
class TagPin:

    def __init__(self, tag_id, tx, ty):
        self.tag_id = tag_id
        self.tx = tx
        self.ty = ty
        
    def get_ID(self):
        return self.tag_id

    def get_tx(self):
        return self.tx
        
    def get_ty(self):
        return self.ty