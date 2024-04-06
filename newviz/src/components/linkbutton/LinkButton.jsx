import React from 'react';
import { Button } from '@mui/material';
import { Link } from 'react-router-dom';

const LinkButton = ({ color, href, label, sx }) => {
  return (
    <Button color={color} component={Link} to={href} sx={sx}>
      {label}
    </Button>
  );
};

export default LinkButton;
